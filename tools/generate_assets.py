#!/usr/bin/env python3
"""Generate matching SDF worlds / occupancy maps from one geometry definition."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RESOLUTION = 0.05
WIDTH, HEIGHT = 400, 300
# x, y, length, width, height: walls plus two shelves, top/bottom passages.
BOXES = [(10,.1,20,.2,1), (10,14.9,20,.2,1), (.1,7.5,.2,15,1),
         (19.9,7.5,.2,15,1), (7,7.5,2,7,1.2), (13,7.5,2,7,1.2)]
STATIONS = [('A01','PICKUP',3,3,0), ('A02','PICKUP',3,12,0),
            ('B01','DROPOFF',17,3,math.pi), ('B02','DROPOFF',17,12,math.pi),
            ('C01','BUFFER',10,12.5,0), ('HOME','HOME',2,7.5,0)]

def text(parent, name, value):
    ET.SubElement(parent, name).text = str(value)

def box(world, name, x, y, sx, sy, sz, collision=True, dynamic=False):
    model = ET.SubElement(world, 'model', name=name)
    text(model, 'static', 'false' if dynamic else 'true')
    text(model, 'pose', f'{x} {y} {sz/2} 0 0 0')
    link = ET.SubElement(model, 'link', name='body')
    if dynamic:
        text(link,'gravity','false')
        inertial=ET.SubElement(link,'inertial'); text(inertial,'mass',100.)
        inertia=ET.SubElement(inertial,'inertia')
        for key,value in [('ixx',100*(sy*sy+sz*sz)/12),('iyy',100*(sx*sx+sz*sz)/12),('izz',100*(sx*sx+sy*sy)/12),('ixy',0),('ixz',0),('iyz',0)]: text(inertia,key,value)
        velocity=ET.SubElement(model,'plugin',filename='gz-sim-velocity-control-system',name='gz::sim::systems::VelocityControl')
        text(velocity,'topic','/model/scenario_box/cmd_vel')
    for kind in (['collision','visual'] if collision else ['visual']):
        obj = ET.SubElement(link, kind, name=kind)
        text(ET.SubElement(ET.SubElement(obj,'geometry'),'box'),'size',f'{sx} {sy} {sz}')
        if kind == 'visual':
            mat = ET.SubElement(obj,'material')
            color = '.17 .28 .36 1' if collision else '.2 .75 .5 1'
            text(mat,'ambient',color); text(mat,'diffuse',color)

def generate_world(name, boxes):
    sdf=ET.Element('sdf',version='1.9'); world=ET.SubElement(sdf,'world',name='warehouse')
    physics=ET.SubElement(world,'physics',name='physics',type='ignored')
    text(physics,'max_step_size',.002); text(physics,'real_time_factor',1.0)
    for file, plugin in [('physics','Physics'),('user-commands','UserCommands'),
                         ('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
        p=ET.SubElement(world,'plugin',filename=f'gz-sim-{file}-system',name=f'gz::sim::systems::{plugin}')
        if file=='sensors': text(p,'render_engine','ogre2')
    truth=ET.SubElement(world,'plugin',filename='libamr_truth_system.so',name='amr::TruthSystem')
    text(truth,'model_name','amr')
    light=ET.SubElement(world,'light',name='sun',type='directional')
    text(light,'pose','0 0 10 0 0 0'); text(light,'diffuse','.8 .8 .8 1'); text(light,'direction','-.5 .1 -1')
    box(world,'floor',10,7.5,24,19,.1)
    # Floor top at z=0, map and wheel contact share that origin.
    world.find("model[@name='floor']/pose").text='10 7.5 -0.05 0 0 0'
    for i,b in enumerate(boxes): box(world,f'obstacle_{i}',*b)
    for station,_,x,y,_ in STATIONS: box(world,'station_'+station,x,y,.45,.45,.008,False)
    # Movable collision fixture parked outside warehouse until a scenario positions it.
    box(world,'scenario_box',-5,-5,.6,.6,1,dynamic=True)
    for fixture,sx,sy in [('enclosure_west',.2,2.4),('enclosure_east',.2,2.4),('enclosure_north',2.4,.2),('enclosure_south',2.4,.2)]:
        box(world,fixture,-8,-8,sx,sy,1)
    ET.indent(sdf)
    path=ROOT/'src/amr_simulation/worlds'/f'{name}.sdf'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(ET.tostring(sdf,encoding='unicode')+'\n',encoding='utf-8')

def generate_map(name, boxes):
    # P5 PGM: top row is high world y; no unknown cells in analytic fixed map.
    pixels=bytearray()
    for row in range(HEIGHT):
        y=(HEIGHT-row-.5)*RESOLUTION
        for col in range(WIDTH):
            x=(col+.5)*RESOLUTION
            occupied=any(abs(x-bx)<=sx/2 and abs(y-by)<=sy/2 for bx,by,sx,sy,_ in boxes)
            pixels.append(0 if occupied else 254)
    folder=ROOT/'src/amr_navigation/maps'; folder.mkdir(parents=True,exist_ok=True)
    (folder/f'{name}.pgm').write_bytes(f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode()+pixels)
    (folder/f'{name}.yaml').write_text(f'map_id: {name}\nimage: {name}.pgm\nmode: trinary\nresolution: {RESOLUTION}\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',encoding='utf-8')

def main():
    for name in ['mini_warehouse','warehouse']:
        generate_world(name,BOXES)
    generate_map('warehouse_v1',BOXES)
    # 0.8m passage with a bypass, identical map/world geometry.
    narrow=BOXES+[(10,11.6,4,.8,1),(10,13.2,4,.8,1)]
    generate_world('narrow_aisle',narrow); generate_map('narrow_aisle',narrow)
    folder=ROOT/'src/amr_mission/config'; folder.mkdir(parents=True,exist_ok=True)
    lines=['schema_version: 1','map_id: warehouse_v1','frame_id: map','stations:']
    for sid,role,x,y,yaw in STATIONS:
        lines += [f'  - station_id: {sid}',f'    type: {role}',
                  f'    pose: {{x: {x:.6f}, y: {y:.6f}, yaw: {yaw:.6f}}}',
                  '    docking_mode: nav_only','    load_timeout_s: 30.0','    unload_timeout_s: 30.0']
    (folder/'stations.yaml').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    (folder/'stations_narrow.yaml').write_text('\n'.join(lines).replace('map_id: warehouse_v1','map_id: narrow_aisle')+'\n',encoding='utf-8')
    (ROOT/'src/amr_navigation/maps/metadata.yaml').write_text('schema_version: 1\nmap_id: warehouse_v1\nsource: tools/generate_assets.py\nframe_id: map\nresolution: 0.05\n',encoding='utf-8')
    print('Generated worlds, fixed maps and six stations.')

if __name__=='__main__': main()
