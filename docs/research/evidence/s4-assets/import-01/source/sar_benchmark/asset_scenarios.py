"""Private asset composition; distinct authored layouts, not a held-out corpus."""


def asset_layout(environment):
    def box(name, center, size, material="Concrete", mesh=None):
        result={"id":name,"center_m":center,"size_m":size,"material":material}
        if mesh:result["mesh"]=mesh
        return result
    if environment=="indoor":
        # Two damaged office bays opening onto a longitudinal corridor. The
        # broken partition forces a lateral view change from the entrance.
        boxes=[
            box("north_masonry",[3,3.7,1.3],[7.3,.24,2.6],"Brick"),
            box("south_masonry",[3,-3.7,1.3],[7.3,.24,2.6],"Brick"),
            box("east_fracture",[6.4,0,1.3],[.24,7.5,2.6],"Brick","fractured_wall"),
            box("west_bay",[-.7,2.5,1.3],[.24,2.6,2.6],"Brick"),
            box("west_bay_lower",[-.7,-2.5,1.3],[.24,2.6,2.6],"Brick"),
            box("search_partition",[2.6,.4,1.3],[.24,2.5,2.6],"Brick","fractured_wall"),
            box("office_divider",[4.7,1.9,1.3],[3.2,.22,2.6],"Brick"),
            box("fallen_ceiling",[4.9,2.8,.17],[1.9,1.15,.34],"Concrete","fallen_slab"),
            box("masonry_heap",[2.65,1.5,.17],[1.,.8,.34],"Debris","rubble_heap"),
            box("entry_debris",[.8,-2.65,.13],[1.1,.85,.26],"Debris","rubble_heap"),
            box("office_crate",[5.7,-2.8,.45],[.8,.75,.9],"wooden_crate_01","wooden_crate_01"),
            box("damaged_chair",[4.7,-1.9,.48],[.55,.6,.96],"painted_wooden_chair_02","painted_wooden_chair_02"),
        ]
        target=[4.5,-.1]
    elif environment=="outdoor":
        # Open service courtyard: collapsed facade on the north, a disconnected
        # loading island south-east and an L-shaped access aisle. No room shell.
        boxes=[
            box("north_collapsed_facade",[3.,3.65,1.3],[7.2,.3,2.6],"Brick","fractured_wall"),
            box("retaining_south",[3.,-3.9,.35],[7.4,.3,.7],"Concrete"),
            box("east_loading_wall",[6.5,1.5,.7],[.3,4.7,1.4],"Brick","fractured_wall"),
            box("west_retaining",[-.7,1.9,.45],[.25,3.8,.9],"Concrete"),
            box("island_debris",[2.8,-.9,.28],[1.7,1.3,.56],"Debris","rubble_heap"),
            box("jersey_barrier",[2.7,.55,.55],[.52,1.65,1.1],"concrete_road_barrier_02","concrete_road_barrier_02"),
            box("loading_crate",[5.5,-2.9,.5],[1.,1.,1.],"wooden_crate_01","wooden_crate_01"),
            box("slab_stack",[4.9,2.85,.22],[1.8,1.1,.44],"Concrete","fallen_slab"),
            box("north_heap",[2.1,2.9,.2],[1.3,1.,.4],"Debris","rubble_heap"),
            box("foreground_heap",[.5,-2.7,.12],[1.2,.85,.24],"Debris","rubble_heap"),
        ]
        target=[4.7,1.6]
    else:raise ValueError("unknown environment")
    return boxes,target


MESH_IDS=("fractured_wall","rubble_heap","fallen_slab","exposed_rebar",
          "concrete_road_barrier_02","painted_wooden_chair_02","wooden_crate_01")
