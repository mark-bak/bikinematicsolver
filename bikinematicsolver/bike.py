#Library imports
import csv

from dijkstar import Graph, find_path #type: ignore
import numpy as np #type: ignore

#Solver imports
from bikinematicsolver.kinematic_solver_scipy_min import Kinematic_Solver_Scipy_Min
from bikinematicsolver.dtypes import Pos,Link,Point
import bikinematicsolver.geometry as g

class Bike():
    def __init__(self,data):

        #Input bike geo
        self.points = {}
        self.links = {}
        self.shock = None
        self.params = self._load_params(data)
        self._parse_input_data(data)

        #Derived bike geo
        self.kinematic_loop_points = []
        self.end_eff_points = []
        self.static_points = []
        self.chainline = []

        self.find_kinematic_loop()
        self.find_static_points()
        self.find_end_eff_points()
        self.find_chainline()
           
        self.kinematic_solver = None #Added later in set_kinematic_solver

        #Currently adds a solver that uses Scipy minimisation fcns to numerically solve
        #linkage constraint eqns, however more solvers will likely need to be created for
        #single pivots, and maybe a freudenstein eqn 4-bar solver could also be made
        default_Solver = Kinematic_Solver_Scipy_Min
        self.set_kinematic_solver(default_Solver)

        #Output data
        self.solution = {}

    def load_param(self,param_name):
        try:
            ret = self.params[param_name]
        except:
            ret = 0
        return ret

    def _load_params(self,data): #so grim needs refactor ASAP as with all input
        try:
            ret = data['params']
        except:
            ret = {}
        return ret

    ##Functions to process input data
    def _parse_input_data(self,data):
        """
        This fcn takes input data, and populates the self.points{} and self.links{} dictionaries with point and
        link data represented as nametdtuples
        """  
        if 'points' in data:
            for point_name,point in data['points'].items():
                self.points[point_name] = Point(**point)           

        if 'links' in data:
            for link_name,link in data['links'].items():
                L = Link(link['a'],link['b'])
                if link_name == data['shock']:
                    self.shock = L
                else:
                    self.links[link_name] = L

    def find_kinematic_loop(self):
        """
        Finds the kinematic loop or linkage of the suspension and adds the point names (str format)
        to the list self.kinemamatic_loop_points[]
        Uses shortest path algorithm to find path between two points of type 'ground', along the minimum
        number of links
        """  
        grounds = [name for name in self.points
                  if self.points[name].type == "front_wheel"
                  or self.points[name].type == "ground"]

        #Create graph for shortest path 
        g = Graph(undirected=True)
        for name in self.points: # add nodes
            g.add_node(name)
        for name,link in self.links.items(): # add links
            g.add_edge(link.a,link.b,1)
        
        path = None
        #Loop between grounds, and find the ones connected by links
        for i in range(len(grounds)):
            for j in range(len(grounds)):
                if i !=j and i<j:
                    try:
                        path = find_path(g,grounds[i],grounds[j]) #Fnd shortest path betweeen these two grounds - almost 100% sure this always gives kin. path 
                        self.kinematic_loop_points  = path.nodes
                    except:
                        pass
        
        if path is None:
            rear_wheel_name = [name for name in self.points if self.points[name].type == "rear_wheel"]
            for i in range(len(grounds)):
                try:
                    path = find_path(g,grounds[i],rear_wheel_name[0])
                    path_list = path.nodes
                    path_list.append(grounds[i])
                    print(path_list)
                except:
                    pass
        
        return path

    def find_static_points(self):
        """
        Populates the list of point names (str) that do not move relative to frame in self.end_static_points[]
        """   
        self.static_points = [name for name in self.points
                              if self.points[name].type == "front_wheel"
                              or self.points[name].type == "ground"
                              or self.points[name].type == "bottom_bracket"]

    def find_end_eff_points(self):
        """
        Populates the list of point names (str) that are attached to kinematic loop in self.end_eff_points[]
        """
        self.end_eff_points = [name for name in self.points 
                               if not name in self.kinematic_loop_points 
                               and not name in self.static_points]

    def find_chainline(self):
        """
        Populates the list of chainline cogs, with tuples of form (point_name,pitch_circ_dia), specifiying the center point and pcd of the cogs on chainline.
        Input teeth should be a tuple of form (teetg_chainring, teeth_idler, teeth_cassette) - where the idler pcd can be omitted if unused
        Output list is ordered from chainring - idler - rear cassette
        """
        #Calculate pcd from https://www.mathopenref.com/polygonradius.html
        if not 'teeth' in self.params:
            return
        n = np.array(self.params['teeth'],dtype = float)
        link_len = 12.7
        pitch_circle_dia = link_len / np.sin( np.pi / n  )

        for point in self.points.values():
            if point.type == 'bottom_bracket':
                self.chainline.insert(0,(point.name,pitch_circle_dia[0]))
            if point.type == 'rear_wheel':
                self.chainline.append((point.name,pitch_circle_dia[-1]))
            if point.type == 'idler':
                self.chainline.insert(1,(point.name,pitch_circle_dia[1])) #one idler only for now - if some crazy person makes bike with two idlers this will need fixed       

    def set_kinematic_solver(self,Solver):
        """
        Adds the kinematic solver which can be used to calculate suspension motion
        """
        self.kinematic_solver = Solver(
            self.points,
            self.links,
            self.kinematic_loop_points,
            self.end_eff_points)

    def get_suspension_motion(self,travel,name):
        """
        Runs the kinematic solver class attached to this instance, and returns the suspension motion
        throughout the specified travel in self.solution
        """
        try:
            sol = self.kinematic_solver.solve_suspension_motion(
                travel,
                self.points,
                self.links,
                self.kinematic_loop_points,
                self.end_eff_points)

            self.solution[name] = sol
        except:
            print('Unable to solve -> idk why, you probably dont have a functional linkage')

    def calculate_suspension_characteristics(self,sol_name):
        """
        Calculates derived suspension characteristics such as leverage ratio, from a solution result, sol_name.

        Directly modifies self.solution[sol_name]

        pretty grim functon - needs a refactor tbh
        """
        if not sol_name in self.solution:
            print("no valid solution with that sol_name")
            return

        sol = self.solution[sol_name]
        size = np.size(list(sol.values())[0].x) 

        ##Static Points
        for point_name in self.static_points:
            r = self.populate_static_point(sol, size, point_name)
            self.solution[sol_name][point_name] = r
        
        ##Instant centre
        if len(self.kinematic_loop_points) < 4:
            #If single pivot - Ic is constant
            Instant_Centre = self.populate_static_point(sol,size,self.kinematic_loop_points[0])
        else:
            #n-bar soln
            a1 = sol[ self.kinematic_loop_points[0] ]
            a2 = sol[ self.kinematic_loop_points[1] ]
            b1 = sol[ self.kinematic_loop_points[-2] ]
            b2 = sol[ self.kinematic_loop_points[-1] ] 

            Instant_Centre = g.find_intersection(a1,a2,b1,b2)

        ##Rear wheel 'travel'
        rw_name = None
        for name in self.points:
            if self.points[name].type == 'rear_wheel':
                rw_name = name
        if rw_name is not None:
            rw_norm = self.offset_to_zero(sol,size,rw_name)
            Vertical_Travel = rw_norm.y
            Axle_Path_X = rw_norm.x
        else:
            Vertical_Travel = np.zeros(size)
            Axle_Path_X = np.zeros(size)

        ##Shock Length and Leverage Ratio
        if self.shock is not None:
            #Find shock point names
            s_a_name = self.shock.a
            s_b_name = self.shock.b
            Shock_Length = self.calc_distance(sol[s_a_name] , sol[s_b_name])
            Leverage_Ratio = self.calc_derivative(Vertical_Travel,Shock_Length)
            #Add to solution
        else:
            Shock_Length = np.zeros(size)
            Leverage_Ratio = np.zeros(size)

        ##Anti Squat:
        if len(self.chainline) > 1:
            input_name = self.chainline[-2][0]
            inp_rad = self.chainline[-2][1] * 0.5

            output_name = self.chainline[-1][0]
            out_rad = self.chainline[-1][1] * 0.5
            #Find chainline and IFC
            IFC_x = np.zeros(size)
            IFC_y = np.zeros(size)
            Cline1_x = np.zeros(size)
            Cline1_y = np.zeros(size)
            Cline2_x = np.zeros(size)
            Cline2_y = np.zeros(size)
            #loop through and find:
            #- upper circle intersection points for each step in solution
            #- ifc: intersection of chainline and 'effective swingarm' line between rw and instant centre
            for i in range(size): 
                #grim processing, maybe need to optimizez datatypes a bit
                out_pos = Pos(sol[output_name].x[i],sol[output_name].y[i])
                inp_pos = Pos(sol[input_name].x[i],sol[input_name].y[i])
                IC = Pos(Instant_Centre.x[i],Instant_Centre.y[i])
                #find all possible lines tangent to the two circles
                possible_clines = g.find_common_circle_tangent(inp_pos,
                                                        inp_rad,
                                                        out_pos,
                                                        out_rad)
                #find which is the 'upper' solution
                cline_pts = g.find_upper_tangent_points(possible_clines,inp_pos,out_pos)
                #find ifc (intersection)
                res = (g.find_intersection(cline_pts[0],cline_pts[1],out_pos,IC))
                
                #more grim processing - cant wait to rewrite this nicer :)
                Cline1_x[i] = cline_pts[0].x
                Cline1_y[i] = cline_pts[0].y
                Cline2_x[i] = cline_pts[1].x
                Cline2_y[i] = cline_pts[1].y
                IFC_x[i] = res.x
                IFC_y[i] = res.y           

            #Results in expected data format
            IFC = Pos(IFC_x,IFC_y)
            CLINE_1 = Pos(Cline1_x,Cline1_y)
            CLINE_2 = Pos(Cline2_x,Cline2_y)

            #Find tyre contact and AS line
            wheel_rad = float(self.load_param('wheel_size')) * 25.4 * 0.5
            Tyre_Contact = Pos(sol[rw_name].x, sol[rw_name].y - np.ones(size) * wheel_rad)
            fw_name = None # needs moved and tidied
            for name in self.points:
                if self.points[name].type == 'front_wheel':
                    fw_name = name        
            AS_x = sol[fw_name].x
            AS_y = np.add( Tyre_Contact.y , (np.subtract(AS_x, Tyre_Contact.x) / np.subtract(IFC.x , Tyre_Contact.x)) * np.subtract(IFC.y,Tyre_Contact.y) )

            AS_P = Pos(AS_x,AS_y) 

            ground_y = Tyre_Contact.y[0]
            h_cog = float(self.load_param('cog_height')) * np.ones(size)
            h_AS = AS_P.y - ground_y * np.ones(size) 

            AS_Percent = 100 *  (h_AS / h_cog) 
        else:
            IFC = np.zeros(size) 
            CLINE_1 = np.zeros(size) 
            CLINE_2 = np.zeros(size)
            AS_P = np.zeros(size) 
            Tyre_Contact = np.zeros(size)
            AS_Percent = np.zeros(size) 
                       

        #Populate solution
        self.solution[sol_name]['ShockLength'] = Shock_Length
        self.solution[sol_name]['LeverageRatio'] = Leverage_Ratio
        self.solution[sol_name]['VerticalTravel'] = Vertical_Travel
        self.solution[sol_name]['AxlePathX'] = Axle_Path_X
        self.solution[sol_name]['InstantCentre'] = Instant_Centre
        self.solution[sol_name]['IFC'] = IFC
        self.solution[sol_name]['CLINE_1'] = CLINE_1
        self.solution[sol_name]['CLINE_2'] = CLINE_2
        self.solution[sol_name]['AS_Point'] = AS_P
        self.solution[sol_name]['Tyre_Contact'] = Tyre_Contact
        self.solution[sol_name]['AntiSquatPercent'] = AS_Percent

    def populate_static_point(self,sol,size,point_name):
        x = np.ones(size) * self.points[point_name].pos[0]
        y = np.ones(size) * self.points[point_name].pos[1]
        return Pos(x,y)

    def offset_to_zero(self,sol,size,point_name):
        #Normalised  path
        y = sol[point_name].y - np.ones(size) * sol[point_name].y[0] # s - I*s[0]
        x = sol[point_name].x - np.ones(size)  * sol[point_name].x[0]
        return Pos(x,y)

    def calc_distance(self,a,b):
        dx = np.subtract(a.x , b.x)
        dy = np.subtract(a.y , b.y)
        L = np.linalg.norm(np.vstack([dx,dy]),axis=0)
        return L

    def calc_derivative(self,x,y):
        dydx = np.abs(np.gradient(x, y))
        return dydx

    def find_ifc(self):
        pass

    ##Data i/o functions
    def save_solution_csv(self,solution_name,filename):
        """
        Fcn to save solution data, solution_name, as a csv called filename
        """
        #Ensure correct filetype
        ind = filename.find('.')
        if ind != -1:
            filename = filename[0:ind]
        filename = "Results\\{}.csv".format(filename)

        #Save data
        with open(filename, 'w', newline ='') as f:
            writer = csv.writer(f, delimiter=',',
            quotechar='|', quoting=csv.QUOTE_MINIMAL)
            for point_name in self.solution[solution_name]:
                if isinstance(self.solution[solution_name][point_name],Pos):
                    writer.writerow([point_name+'_x']+list(self.solution[solution_name][point_name].x))
                    writer.writerow([point_name+'_y']+list(self.solution[solution_name][point_name].y))
                else:
                    writer.writerow([point_name]+list(self.solution[solution_name][point_name]))
                    pass



