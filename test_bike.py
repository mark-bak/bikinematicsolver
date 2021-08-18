from bikinematicsolver.bike import Bike
import json
import numpy

def test_no_params():
    data = {}
    travel = 200
    b = Bike(data)
    b.get_suspension_motion(travel,'test')
    b.calculate_suspension_characteristics('test')

def test_params():
    with open('test_data.json') as json_data:
        data = json.load(json_data)
        b = Bike(data)
        travel = 200
        b.get_suspension_motion(travel,'test')
        assert isinstance(b.solution['test']['Upper_Pivot'].x,numpy.ndarray)
        b.calculate_suspension_characteristics('test')
        assert isinstance(b.solution['test']['LeverageRatio'],numpy.ndarray)
        print(b.solution['test']['AntiSquatPercent'])
        print(b.solution['test']['LeverageRatio'])

