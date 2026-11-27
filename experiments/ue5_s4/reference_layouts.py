"""Art direction for the damaged records office and rail-side distribution depot.

Positions are FLU metres. Kit walls extend from local X=-3 to their right jamb at 0.
The catalogue pivot is shared by the matching wall aperture and its joinery.
"""
import math

import cv2
import numpy as np

from .site_architecture import prism, slab, solid

OFFICE = "modular_urban_apartments_facade"
FACTORY = "modular_factory_facade"
DUCT = "modular_airduct_rectangular_01"


def bay(put, kit, opening, pivot, yaw, width=3, height=3):
    source_width = 6 if "garage" in opening else 3
    put(kit, ("wall_"+opening, opening), pivot, yaw, (width/source_width,1,height/3))


def spoil(layers, heightmap, center, size, heading, seed):
    """Taper a patch of registered scan displacement into the foot of a break."""
    mesh = layers["rubble"]
    rng = np.random.default_rng(seed)
    c,s = math.cos(heading),math.sin(heading)
    start = len(mesh.vertices)
    rings = 11
    samples = 40
    rough_edge = rng.uniform(.84,1.,samples)
    for i in range(rings+1):
        for j in range(samples):
            a = 2*math.pi*j/samples
            r = (i+.025)/(rings+.025)*rough_edge[j]
            dx,dy = .5*size[0]*r*math.cos(a),.5*size[1]*r*math.sin(a)
            u,v = .16*seed+dx/2,.22*seed+dy/2
            h,w = heightmap.shape
            sample = heightmap[int((1-v)%1*h)%h,int(u%1*w)%w]/255
            # A low skirt of dust under discontinuous piles of aggregate.
            ridge = .65+.35*math.sin(3*a+.6*seed+r*4)
            z = size[2]*(1-i/rings)**1.15*ridge*(.25+1.55*sample)
            mesh.vertices.append([center[0]+c*dx-s*dy,center[1]+s*dx+c*dy,center[2]+z])
            mesh.uvs.append([u,v])
    for i in range(rings):
        for j in range(samples):
            a = start+i*samples+j
            b = start+i*samples+(j+1)%samples
            mesh.faces.extend([[a,a+samples,b+samples],[a,b+samples,b]])
    mesh.faces.extend([[start,start+j,start+j+1] for j in range(1,samples-1)])
    for _ in range(17):
        a = rng.uniform(0,math.tau)
        r = rng.uniform(.24,1.14)
        u,v = size[0]*.50*r*math.cos(a),size[1]*.50*r*math.sin(a)
        x,y = center[0]+c*u-s*v,center[1]+s*u+c*v
        size_x = rng.uniform(.09,.31)
        layer = layers["broken_brick_wall" if rng.random()<.66 else "cracked_concrete"]
        layer.shard((x,y,center[2]+.19*size[2]*max(0,1-r)),
                    (size_x,size_x*rng.uniform(.45,.75),size_x*rng.uniform(.27,.55)),a)


def office(put, layers):
    # Windowed offices on both sides, backing onto a two-storey records wing.
    for i,opening in enumerate(("window_centered_double_01","window_centered_large_01","window_centered_small_01")):
        bay(put,OFFICE,opening,(1.78+2.44*i,3.70,0),0,2.44)
        bay(put,OFFICE,opening,(-.66+2.44*i,-3.70,0),180,2.44)
    for i in range(4):
        for z in (0,3):
            opening = "door_centered_small_01" if i==1 and z==0 else "window_centered_double_01"
            bay(put,OFFICE,opening,(8.65,-5.4+i*3,z),-90)
    for x in (11.65,14.65):
        for z in (0,3):
            bay(put,OFFICE,"window_centered_large_01",(x,6.6,z),0)
    concrete,metal = layers["cracked_concrete"],layers["Metal"]
    solid(concrete,8.6,14.7,-5.45,6.65,2.87,3.)
    solid(concrete,8.52,14.77,-5.55,6.73,6.,6.18)
    solid(layers["plastered_wall_02"],14.5,14.7,-5.4,6.6,0,6.)
    # The floor finish survives inside the offices; a torn irregular strip
    # along the collapsed north bay exposes the substrate and the debris fan.
    edges = [(-.75,3.2),(.55,3.44),(1.65,2.85),(2.54,1.64),(3.36,1.51),
             (4.24,.94),(5.28,1.49),(6.51,.94)]
    for (x,y),(xx,yy) in zip(edges,edges[1:]):
        slab(layers["linoleum_brown"],[(x,-3.59),(xx,-3.59),(xx,yy),(x,y)],.008,.008)
    # Ceiling slab, open plenum and suspended services terminate at the same fracture.
    edge = [(-.81,2.32),(.44,2.04),(1.05,1.59),(1.71,1.83),(2.53,1.23),
            (3.01,.62),(3.54,.75),(4.26,-.11),(4.93,.24),(5.56,-.76),(6.6,-.39)]
    for (x,y),(xx,yy) in zip(edge,edge[1:]):
        slab(concrete,[(x,-3.82),(xx,-3.82),(xx,yy),(x,y)],3.14,.21)
        for f in (.18,.57,.84):
            px,py=x+(xx-x)*f,y+(yy-y)*f
            metal.tube([(px,py-.04,3.04),(px+.01,py+.24,3.01),(px+.05,py+.42,2.82)],.011,7)
    for i in range(4):
        put(DUCT,DUCT+"_tripple_01",(-.8+1.8*i,-1.84,2.65),90)
        put(DUCT,DUCT+"_brace_01",(-.58+1.8*i,-1.84,2.404),90)
    # Suspended ceiling remains above reception. At the break the plasterboard
    # is missing from the tee grid and two steel hangers have folded down.
    for x in (-.70,.51,1.72,2.93,4.14,5.35):
        solid(metal,x,x+.025,-3.55,-2.24,2.47,2.50)
        for j in range(2):
            y=-3.54+.62*j
            if (x>1.7 and j==1) or x>4.2:
                continue
            solid(layers["plastered_wall_02"],x+.03,x+1.19,y+.02,y+.60,2.485,2.508)
        metal.tube([(x,-2.25,2.94),(x,-2.25,2.47)],.004,5)
    metal.tube([(2.95,-2.24,2.48),(3.56,-2.22,2.43),(4.13,-2.15,2.14)],.014,5)
    metal.tube([(5.35,-2.25,2.48),(5.57,-2.19,2.06),(5.68,-2.21,1.89)],.012,5)
    put(DUCT,DUCT+"_corner_02",(2.3,1.40,2.76),180,roll=12)
    put(DUCT,DUCT+"_transition_double_01",(2.30,1.39,2.72),0,pitch=-14)
    # A records station and an abandoned reception desk. The laptop's lid and
    # drawers keep the relative transforms in the source prop.
    put("classic_laptop",(),(5.69,.15,.79),90,scale=(.78,.78,.78))
    put("office_notepads","office_notepads_a4_stack",(5.63,1.04,.792),7)
    put("office_notepads","office_notepads_yellow_pad",(-.04,-2.45,.792),-15)
    put("drawer_cabinet",(),(.66,3.24,0),180)
    put("drawer_cabinet",(),(5.58,2.96,0),180)
    for i,(x,y) in enumerate(((1.2,-2.90),(1.46,-2.84),(1.68,-3.08))):
        put("office_notepads","office_notepads_a4_"+("a","b","c")[i],(x,y,.012),31*i-38)
    # Plaster still clings to the brick partition below its broken top line.
    profile=[(-.83,2.53),(-.53,2.35),(-.29,1.69),(.02,1.44),(.21,.85),
             (.42,.76),(.64,.60),(.91,.92),(1.18,1.27),(1.48,2.02),(1.62,2.42)]
    for (y,z),(yy,zz) in zip(profile,profile[1:]):
        face=[(2.466,y,.11),(2.466,y,z),(2.466,yy,zz),(2.466,yy,.11)]
        layers["plastered_wall_02"].quad(*face)
    solid(metal,2.447,2.47,-.805,-.215,1.66,1.82)
    solid(metal,8.37,8.65,-.96,-.14,2.58,2.83)
    # Corridor and upper-storey silhouettes continue beyond the compound.
    for i in range(6):
        for z in (0,3):
            bay(put,OFFICE,"window_centered_double_01",(1+3*i,15,z),0)
    solid(concrete,-2,16,15,20,6.,6.14)


def depot(put,layers):
    # Three six-metre loading bays and transoms share a dock slab. The middle
    # door is missing, revealing the production floor and overhead bridge crane.
    for i in range(3):
        pivot=(8.82,-8.65+6*i,.48)
        if i==1:
            put(FACTORY,"wall_door_garage_centered_01",pivot,-90)
        else:
            bay(put,FACTORY,"door_garage_centered_01",pivot,-90,6)
    for i in range(6):
        y=-8.65+3*i
        bay(put,FACTORY,"window_tall_large_01",(8.82,y,3.48),-90)
        put(FACTORY,"cornice01_standard_standard_01",(8.81,y,6.48),-90)
        put(FACTORY,"wall_pier_standard_01",(8.8,y,.48),-90,scale=(1,1,2))
    for x in (11.82,14.82,17.82):
        bay(put,FACTORY,"window_tall_large_01",(x,9.35,.48),0)
        bay(put,FACTORY,"window_centered_double_01",(x,9.35,3.48),0)
    concrete,steel = layers["cracked_concrete"],layers["Metal"]
    for y in (-5.7,.3,6.3):
        solid(steel,8.55,8.83,y-.39,y+.39,2.87,3.26)
    solid(concrete,7.58,17.94,-8.78,9.48,-.16,.48)
    solid(layers["broken_brick_wall"],17.64,17.90,-8.7,9.4,.48,6.50)
    for y in (-8.70,-2.7,3.3):
        lower=[(8.66,y,6.50),(18.,y,6.50),(18.,y+6,7.72),(8.66,y+6,7.72)]
        prism(layers["corrugated_iron_03"],lower,[(x,v,z+.10) for x,v,z in lower])
        for x in (8.82,17.83):
            steel.tube([(x,y,6.47),(x,y+6,7.66),(x,y+6,6.47)],.075,6)
    put("overhead_crane",(),(12.56,.25,5.33),90)
    put("industrial_storage_cart",(),(5.56,-2.87,.014),165)
    put("hand_truck",(),(5.92,-.37,.016),-27)
    put("portable_generator",(),(3.58,2.29,.018),155)
    put("industrial_storage_cart",(),(8.0,-3.4,.49),90)
    # West and north returns enclose a real service street beyond the search yard.
    for i in range(5):
        for z in (0,3):
            bay(put,FACTORY,"window_centered_double_01",(2+3*i,16,z),0)
    solid(concrete,-1.05,14.08,16.,21.,6.,6.15)
    for x in (-1.01,13.99):
        solid(layers["broken_brick_wall"],x,x+.21,16.,21.,-.18,6.)
    for i in range(3):
        bay(put,FACTORY,"door_garage_centered_01",(-9.5,9.-6*i,0),90,6)
    solid(concrete,-16,-9.50,-9.,9.,3.,3.15)
    # Wheel-worn loading apron and paint alongside the curb.
    solid(layers["road_damaged_2"],-16,8.81,-19,15.7,-.18,-.11)
    for i in range(7):
        solid(layers["Paint"],6.94,7.02,-8.2+2.4*i,-7.0+2.4*i,-.108,-.102)


def assemble(environment,put,layers,root):
    indoor = environment=="indoor"
    solid(layers["asphalt_04"],-60,70,-60,65,-.39,-.19)
    (office if indoor else depot)(put,layers)
    # Pioneer vegetation follows the unmaintained perimeter and pavement joints.
    # Keep the navigable apron and the lit interior clear of foliage cards.
    shrubs = ([(7.32,4.82,.90),(7.81,5.14,.72),(6.90,5.38,1.04),
               (3.20,8.37,1.18),(.13,8.62,.92),(-2.95,8.84,1.05)]
              if indoor else [(7.40,5.42,1.12),(6.75,5.93,.82),(7.87,5.84,1.05),
                              (5.70,7.13,1.33),(2.98,7.52,1.25),(-1.29,8.24,1.43),
                              (-4.2,8.95,1.6),(-8.42,6.19,1.18)])
    for i,(x,y,scale) in enumerate(shrubs):
        put("shrub_04","shrub_04",(x,y,-.18),i*67,(scale,scale,scale))
    if not indoor:
        put("dead_tree_trunk_02",(),(1.85,6.72,.08),27)
        for i,(x,y) in enumerate(((.40,3.32),(.81,3.37),(1.14,3.41),(4.22,-3.60),(4.72,-3.62),(5.55,-3.63))):
            put("weed_plant_02","weed_plant_02_"+("a","c","e")[i%3]+"_LOD0",(x,y,.006),48*i)
    # Windbreak beyond the search boundary. Gaps give views across the street.
    for i,(x,y,s) in enumerate(((-8,12,.90),(-5,19,1.1),(-1,25,1.18),(18,16,1.03),
                                (19,10,.84),(23,-4,1.16),(17,-16,1.0),(-5,-19,.86))):
        put("fir_sapling_medium","fir_sapling_medium_c_LOD0",(x,y,-.18),41+57*i,(s,s,s))
    height = cv2.imread(str(root/"source/rubble/height.img"),cv2.IMREAD_GRAYSCALE)
    if height is None:
        raise ValueError("missing registered rubble displacement")
    fans = ([(2.69,1.49,1.50,1.13,.32,.7),(4.73,2.72,2.87,1.56,.32,-.2),
             (.87,-2.67,1.61,1.14,.25,.1),(5.94,1.69,1.72,1.17,.22,.8)]
            if indoor else [(2.80,-.90,1.86,1.42,.47,.08),(.52,-2.7,1.6,1.05,.23,-.2),
                            (2.1,2.94,1.64,1.38,.36,.7),(4.87,2.87,2.14,1.31,.30,.2)])
    for i,(x,y,w,d,z,a) in enumerate(fans):
        spoil(layers,height,(x,y,.010),(w,d,z),a,i+1)
