from bikinematicsolver.dtypes import Pos
import bikinematicsolver.geometry as bg

def test_intn():
    a1 = Pos(1,1); a2 = Pos(2,2); b1 = Pos(1,2); b2 = Pos(2,1) 
    result = bg.find_intersection(a1,a2,b1,b2)
    assert result.x == 1.5
    assert result.y ==1.5

