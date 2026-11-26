"""Original CC0 metric architecture for the two S4 rescue sites.

All coordinates are in the same forward/left/up metres as the robot. Each
material layer is a real mesh, including reveals, slab thickness and doorways.
"""
from collections import defaultdict


def prism(mesh, bottom, top):
    """Close two corresponding convex polygons with outward-facing triangles."""
    for i in range(1, len(top)-1):
        mesh.triangle((bottom[0], bottom[i+1], bottom[i]))
        mesh.triangle((top[0], top[i], top[i+1]))
    for i in range(len(top)):
        j = (i+1) % len(top)
        mesh.quad(bottom[i], bottom[j], top[j], top[i])


def slab(mesh, footprint, top, thickness):
    upper = [(x, y, top) for x, y in footprint]
    prism(mesh, [(x, y, top-thickness) for x, y in footprint], upper)


def solid(mesh, x0, x1, y0, y1, z0, z1):
    slab(mesh, [(x0,y0), (x1,y0), (x1,y1), (x0,y1)], z1, z1-z0)


def lintelled_wall(layers, x, y, width, height, openings, finish, base=0):
    """Facade along Y with a recessed reveal and a clear opening for each bay."""
    wall = layers[finish]
    cursor = y
    for start, end, sill, head in openings:
        solid(wall, x, x+.30, cursor, start, base, height)
        solid(wall, x, x+.30, start, end, head, height)
        if sill > base:
            solid(wall, x, x+.30, start, end, base, sill)
            solid(layers["cracked_concrete"], x-.13, x+.32, start-.08, end+.08, sill-.08, sill)
        cursor = end
    solid(wall, x, x+.30, cursor, y+width, base, height)


def window(layers, x, left, right, sill, head, missing=()):
    # Deep steel frames and recessed dark panes read as openings at rover height.
    frame = layers["Metal"]
    for z in (sill, (sill+head)/2, head):
        solid(frame, x-.04, x+.04, left, right, z-.025, z+.025)
    for i in range(4):
        y = left + (right-left)*i/3
        solid(frame, x-.04, x+.04, y-.025, y+.025, sill, head)
    for i in range(3):
        for j in range(2):
            if (i,j) in missing:
                continue
            y0 = left+(right-left)*i/3+.025
            y1 = left+(right-left)*(i+1)/3-.025
            z0 = sill+(head-sill)*j/2+.025
            z1 = sill+(head-sill)*(j+1)/2-.025
            solid(layers["Window"], x+.07, x+.085, y0, y1, z0, z1)


def office(layers):
    plaster = layers["plastered_wall_02"]
    concrete = layers["cracked_concrete"]
    steel = layers["Metal"]
    # An occupied wing continues beyond the torn eastern wall. Looking through
    # the breach reveals the next room, its doorway and windows at human scale.
    lintelled_wall(layers, 8.1, -4.8, 10.2, 2.94,
                   [(-3.9,-2.5,.92,2.24),(-1.4,-.35,0,2.12),(.55,2.05,.92,2.24),(3.,4.55,.92,2.24)],
                   "plastered_wall_02")
    for a,b in [(-3.9,-2.5),(.55,2.05),(3.,4.55)]:
        window(layers, 8.23,a,b,.92,2.24,missing=((1,0),))
    solid(concrete, 8.4,12., -4.8,5.4,2.8,3.03)
    solid(plaster, 11.8,12., -4.8,5.4,0,5.65)
    solid(plaster, 6.6,12., 5.2,5.4,0,5.65)
    upper = [(y,y+1.55,3.57,4.95) for y in (-3.5,-.1,3.2)]
    lintelled_wall(layers,8.1,-4.8,10.2,5.65,upper,"plastered_wall_02",base=2.94)
    for left,right,sill,head in upper:
        window(layers,8.23,left,right,sill,head)
    # Roof above the corridor survives on the south side. Torn concrete edges
    # lead to the fallen slab in the north bay. The hole admits direct light.
    edge = [(-.85,1.65),(.6,1.55),(1.4,.92),(2.05,1.15),(2.8,.25),
            (3.55,.47),(4.1,-.38),(4.8,.08),(5.5,-.70),(6.55,-.34)]
    for (x0,y0),(x1,y1) in zip(edge,edge[1:]):
        slab(concrete,[(x0,-3.82),(x1,-3.82),(x1,y1),(x0,y0)],2.91,.20)
        for fraction in (.25,.67):
            x=x0+(x1-x0)*fraction
            y=y0+(y1-y0)*fraction
            steel.tube([(x,y-.08,2.79),(x,y+.19,2.76),(x+.035,y+.35,2.59)],.011,6)
    # A remaining plaster finish on the long interior walls, with missing
    # patches along the base that expose the structural brick beneath.
    for y in (-3.566,3.566):
        for left,right,base in [(-.49,1.05,.22),(1.065,2.75,.41),(2.765,4.55,.13),(4.565,6.24,.57)]:
            solid(plaster,left,right,y-.012,y+.012,base,2.55)
        steel.tube([(-.44,y*.988,2.34),(3.5,y*.988,2.34),(3.67,y*.988,2.12),(6.18,y*.988,2.12)],.032,8)
    # Door casing on an existing opening, ceiling cable tray and suspended
    # fixture with actual brackets. Keep the rover corridor clear underneath.
    # Both jambs are represented in the private layout for navigation admission.
    solid(steel,2.48,2.65,-1.97,-.87,2.12,2.20)
    for x in (.65,3.56):
        for dx in (-.36,.36):
            steel.tube([(x+dx,-1.86,2.70),(x+dx,-1.86,2.48)],.012,6)
        solid(steel,x-.49,x+.49,-1.98,-1.74,2.39,2.48)
        solid(layers["Diffuser"],x-.44,x+.44,-1.96,-1.76,2.375,2.40)
    steel.tube([(4.1,.18,2.72),(4.18,.19,2.07),(4.44,.18,1.77),(4.69,.18,1.69)],.014,6)


def depot(layers):
    concrete = layers["cracked_concrete"]
    steel = layers["Metal"]
    brick = layers["broken_brick_wall"]
    iron = layers["corrugated_iron_03"]
    # Continuous street, footways and warehouse shell behind the yard.
    solid(layers["asphalt_04"],-21.,30.,-19.,22.,-.23,-.145)
    solid(concrete,8.55,10.2,-13.,13.,-.145,.08)
    solid(concrete,9.60,10.7,-9.85,9.85,.08,.49)
    bays=[(-8.45,-5.25),(-3.55,-.35),(1.25,4.45),(6.,8.65)]
    lintelled_wall(layers,10.25,-10.10,20.3,3.93,
                   [(a,b,0,3.03) for a,b in bays], "broken_brick_wall")
    for i,(left,right) in enumerate(bays):
        bottom = .95 if i==2 else .48
        solid(iron,10.42,10.49,left,right,bottom,3.04)
        for z in (bottom,bottom+.61,bottom+1.22):
            solid(steel,10.37,10.43,left,right,z,z+.032)
        for y in (left-.06,right+.06):
            solid(concrete,10.05,10.25,y-.065,y+.065,.48,3.21)
        solid(steel,9.14,10.41,left-.19,right+.19,3.12,3.19)
        for y in (left+.16,right-.16):
            steel.tube([(10.22,y,3.79),(9.28,y,3.19)],.026,8)
    # Clerestory band sits above the loading doors. Sawtooth roof cross-section
    # rises toward the north-facing glazing; interiors remain visible at gaps.
    clerestory = [(y+.06,y+2.96,4.15,5.02) for y in (-9.65,-6.35,-3.05,.25,3.55,6.85)]
    lintelled_wall(layers,10.25,-10.10,20.3,5.25,clerestory,"plastered_wall_02",base=3.93)
    for y in (-9.65,-6.35,-3.05,.25,3.55,6.85):
        solid(concrete,10.16,10.33,y,y+3.03,4.01,4.11)
        window(layers,10.38,y+.06,y+2.96,4.15,5.02,missing=((2,1),) if y==.25 else ())
    for y in (-10.,-5.,0.,5.):
        low=[(10.12,y,5.28),(17.9,y,5.28),(17.9,y+4.97,6.54),(10.12,y+4.97,6.54)]
        prism(iron,low,[(x,v,z+.11) for x,v,z in low])
        a,b,c=(10.27,y,5.25),(10.27,y+4.98,5.25),(10.27,y+4.98,6.55)
        iron.triangle([a,c,b])
        iron.triangle([(x+.04,v,z) for x,v,z in (a,b,c)])
    solid(brick,10.25,18.,10.1,10.4,0,5.28)
    solid(brick,10.25,18.,-10.4,-10.1,0,5.28)
    solid(brick,17.8,18.,-10.1,10.1,0,5.28)
    # Former works office across the side street. Recessed windows, floor
    # bands and returns give parallax to lateral rover turns.
    openings=[(-17.9,-16.3,1.05,2.62),(-15.4,-13.8,1.05,2.62),(-12.9,-11.3,1.05,2.62)]
    lintelled_wall(layers,6.85,-18.85,8.6,3.48,openings,"plastered_wall_02")
    lintelled_wall(layers,6.85,-18.85,8.6,7.1,[(a,b,4.24,5.93) for a,b,_,_ in openings],"plastered_wall_02",base=3.48)
    for a,b,lo,hi in openings:
        window(layers,6.98,a,b,lo,hi)
        window(layers,6.98,a,b,4.24,5.93)
    solid(concrete,6.64,11.9,-18.89,-10.21,3.31,3.47)
    solid(concrete,6.62,11.99,-18.94,-10.17,7.05,7.24)
    solid(layers["plastered_wall_02"],6.85,11.8,-10.55,-10.25,0,7.05)
    for y in (-9.85,-4.57,5.19,9.85):
        steel.tube([(10.19,y,5.26),(10.02,y,4.88),(10.02,y,.53),(9.85,y,.39)],.060,10)
    # Bent light standard beside the service road; a physical shade and arm.
    steel.tube([(8.71,-4.42,.06),(8.71,-4.42,4.27),(8.56,-4.42,4.56),(7.85,-4.42,4.62)],.071,10)
    solid(steel,7.53,7.99,-4.56,-4.28,4.54,4.64)
    solid(layers["Diffuser"],7.55,7.97,-4.54,-4.30,4.52,4.55)
    # Strip of asphalt through the near gate. Shallow paint and drain grating
    # stay flush with the drive surface and do not introduce a hidden curb.
    solid(layers["asphalt_04"],-1.03,1.86,-3.85,3.60,.0005,.006)
    for y in (-2.10,-.9,.30,1.50,2.70):
        solid(layers["Paint"],-.58,-.50,y,y+.55,.006,.011)
    for i in range(25):
        solid(steel,1.61,1.80,-2.8+i*.105,-2.76+i*.105,.006,.017)


def bake_site_meshes(mesh_type, derived):
    """Each mesh uses its own surface finish and retains the metric site origin."""
    mapping = {}
    for environment, build in [("indoor",office), ("outdoor",depot)]:
        layers = defaultdict(mesh_type)
        build(layers)
        for finish, mesh in layers.items():
            name = f"site_{environment}_{finish}"
            mesh.save(derived / (name+".obj"))
            mapping[name] = {"environment":environment, "finish":finish}
    return mapping
