from math import cos,sin,sqrt,acos,pi
import tkinter

#https://www.desmos.com/calculator/oqwigwwtvt
'''
(d*cos(b),d*sin(b)) = target position from target distance vector and bearing angle (from x axis)
(d*cos(b)+v*cos(h)*t,d*sin(b)+v*sin(h)*t) = position + speed(v) and target heading (h), note free variable t used to represent time
[x - (d*cos(b))] / (v*cos(h) = [y-(d*sin(b)] / (v*sin(h) = line equation for the targets velocity

(p*cos(k)*t,p*sin(k)*t) = torpedo position at point time(t), p = torpedo speed, k = torpedo heading angle
p*p = (x/t)^2 + (y/t)^2 = circle equation for torpedo impact point to match with targets velocity at specific time t

angles range 180->0->-180, with 135 being ACW of x axis..
vector angle calculation is < 90 degrees from x axis (use visual)
'''


class Vec:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def magnitude(self):
        result = sqrt(self.lensq())
        return result
    
    def dot(self, other):
        """Dot product"""
        result = self.x * other.x + self.y * other.y + self.z * other.z
        return result

    def lensq(self):
        """The square of the length"""
        return self.dot(self)

    def cross(self, other):
        x = self.y * other.z - self.z * other.y
        y = self.x * other.z - self.z * other.x
        z = self.x * other.y - self.y * other.x
        return Vec(x, -y, z)

    def angle(self, other):#p.q = |p||q|cos(a)
        #angle between: {self}, {other} = acos(  a.b / ( ||a|| * ||b|| )  )
        ab = self.dot(other)
        cd = self.magnitude() * other.magnitude()
        try:
            a = acos(ab / cd)
            return acos(ab / cd)
        except:
            return 0#div 0 error probably

    def unit(self):
        m = self.magnitude()
        if m == 0:
            return Vec()
        return Vec(self.x/m, self.y/m, self.z/m)

    def projection(self, other):
        """projecting x{self} as a point, onto  line d{other} = proj (d) x = (x.d / ||d||**2) * d"""
        scalar = self.dot(other) / other.lensq()
        return other*scalar

    def is_orthogonal(self, other):#2 vectors perpendicular
        """a.b = 0, or angle = 90 degrees"""
        return self.dot(other) == 0

    def is_acute(self, other):
        """a.b > 0 angle < 90 degrees"""
        return self.dot(other) > 0

    def is_obtuse(self, other):
        """a.b < 0 angle > 90 degrees"""
        return self.dot(other) < 0

    def __add__(self, other):
        if type(other) in (int, float):
            return Vec(self.x + other, self.y + other, self.z + other)#vector addition y a scale factor
        else:
            return Vec(self.x + other.x, self.y + other.y, self.z + other.z)#vector addition

    def __sub__(self, other):
        if type(other) in (int, float):
            return Vec(self.x - other, self.y - other, self.z - other)#vector subtraction by scale factor
        else:
            return Vec(self.x - other.x, self.y - other.y, self.z - other.z)#vector subtraction

    def __mul__(self, other):
        if type(other) in (int, float):
            return Vec(self.x * other, self.y * other, self.z * other)#scaling the vector
        else:
            return Vec(self.x * other.x, self.y * other.y, self.z * other.z)

    def __repr__(self):
        return str(self)

    def __str__(self):
        return (f"({self.x:.3f}, {self.y:.3f}, {self.z:.3f})")


#globals
OBJECTS = []
VERTICES = [Vec()]
TIME_STEP = 100
WINDOW_DIMENSIONS = Vec(1920,1080,0)
OFFSET = Vec(WINDOW_DIMENSIONS.x/2,WINDOW_DIMENSIONS.y/2,0)
PROJECTILE_SPEED = 22.6356 #44 knots
SCALE = 0.5
LINES_INITIALIZED = False
#gui
font_style = "Arial"
font_size = 13
label_width = 6
'''
from system of equations:
s**2 = x**2 + y**2
x(distance to travel to target) = d.x/t + targetV.x
y(distance to travel to target) = d.y/t + targetV.y
    x = sqrt(36-y^2)
    y = 277/t - 0.5
    t = 761 / (x + 10)
        solving for x in terms of t by substituting y:
            x = sqrt(36-(277/t - 0.5)^2)
        solving t by substituting in x:
            t = 761 / (sqrt(36-(277/t - 0.5)^2) + 10)
            ...
            36-(277/t - 0.5)^2 = (  (761 / t) - 10)^2
            ...
'''
def binary_estimate(distance_vector,velocity_vector,target_speed,estimate=100, difference=999, depth=0, max_depth=100):
    '''
    s = PROJECTILE_SPEED, v = velocity of target
    s^2 * t = v.x^2 * t + v.y^2 * t
    s^2 * t = t(d.x/t + v.x)^2  + t(d.y/t + v.y)^2
    s^2 = (d.x/t + v.x)^2  + (d.y/t + v.y)^2
    f(t) = s^2 - (d.x/t + v.x)^2 - (d.y/t + v.y)^2
        when f(t) = 0 the projectile has hit the target
        so using newtons method to estimate roots of a function
        f'(t) = ((-2*(-a**2-c**2))/(x**3)) - ((-2*a*b-2*c*d)/x**2)
            where a=d.x, b=v.x, c=d.y, d=v.y

    an+1 = an - f(n)/f'(n)
    '''
    x = estimate#time estimate
    s = float(projectile_speed_entry.get())#PROJECTILE_SPEED#speed of projectile m/s
    a = distance_vector.x
    b = velocity_vector.x
    c = distance_vector.y
    d = velocity_vector.y
    f = s**2 - (a/x + b)**2 - (c/x + d)**2
    f_dash = ((-2*(-a**2-c**2))/(x**3)) - ((-2*a*b-2*c*d)/x**2)
    difference = f/f_dash
    print(difference)
    if abs(difference) < 0.1:
        return estimate - difference
    return binary_estimate(distance_vector,velocity_vector,target_speed,estimate - difference, difference, depth+1)
    
   
def calculate_lead(distance_vector,velocity_vector,t):
    '''
    x = sqrt(36-y^2)
    y  = 277/t - 0.5
    t = 761 / (x + 10)
    '''
    y = distance_vector.y/t + velocity_vector.y
    x = sqrt(abs(PROJECTILE_SPEED**2-y**2))
    return Vec(x,y,0)
    

def init_lines():
    canvas.create_line(0,OFFSET.y,WINDOW_DIMENSIONS.x,OFFSET.y,fill='red')#x axis
    canvas.create_line(OFFSET.x,0,OFFSET.x,WINDOW_DIMENSIONS.y,fill='red')#y axis
    VERTICES = [Vec() for x in range(10)]
    for i in range(1,len(VERTICES)):
        VERTICES[i].y *= -1#invert y as +y goes down tkinter window
        v1 = VERTICES[i-1]*SCALE + OFFSET
        v2 = VERTICES[i]*SCALE + OFFSET
        try:
            OBJECTS[i-1] = canvas.create_line(v1.x,v1.y, v2.x,v2.y)
        except IndexError:
            OBJECTS.append(canvas.create_line(v1.x,v1.y, v2.x,v2.y))#init array of line objects


def render():#stationary object and moving camera
    for i in range(1,len(VERTICES)):
        v1 = (VERTICES[i-1])*SCALE + OFFSET
        v2 = (VERTICES[i])*SCALE + OFFSET
        canvas.coords(OBJECTS[i-1],v1.x,v1.y, v2.x,v2.y)#uses line object id to update the lines coordinates
    #root.after(TIME_STEP,render)


def run():
    PROJECTILE_SPEED = float(projectile_speed_entry.get())
    distance = float(distance_entry.get())
    target_speed = float(target_speed_entry.get())
    angle = float(bearing_entry.get()) * pi/180
    velocity_angle = float(target_heading_entry.get()) * pi/180

    target = Vec(distance*cos(angle),distance*sin(angle),0)#cos(x)=a/h, sin(x)=o/h from the horizontal axis

    target_velocity = Vec(target_speed*cos(velocity_angle),target_speed*sin(velocity_angle),0)

    time_estimate = distance/PROJECTILE_SPEED
    time_to_target = binary_estimate(target,target_velocity,target_speed,time_estimate)
    
    #print('\ntime:',time_to_target,'s')
    target_lead = calculate_lead(target,target_velocity,time_to_target) 
    #print('lead vector:',target_lead)
    solution_angle = target_lead.unit().angle(Vec(1,0,0))*180/pi
    #print(solution_angle*180/pi,'degrees from x axis\n')
    projectile_lead_angle_result_label['text']=f'{(solution_angle):.2f} degrees from +ve x axis'
    
    try:
        VERTICES[1]=(target)
        VERTICES[2]=(target+target_velocity*time_to_target)
        VERTICES[3]=(Vec()) #use to draw target_lead vector from 0,0
        #VERTICES[4]=(target_lead * time_to_target)
    except IndexError:
        VERTICES.append(target)
        VERTICES.append(target+target_velocity*time_to_target)
        VERTICES.append(Vec()) #use to draw target_lead vector from 0,0
        #VERTICES.append(target_lead * time_to_target)
    for i in range(len(VERTICES)):
        VERTICES[i].y *= -1#invert y as +y goes down tkinter window
    render()
    #print(VERTICES)
    

root = tkinter.Tk()
canvas = tkinter.Canvas(root,width=700,height=700)

#GUI buttons
update_button = tkinter.Button(canvas, text="RUN",command=run)
update_button.grid(row=0,column=0)
#GUI entry elements
distance_entry = tkinter.Entry(canvas, width=label_width) 
distance_entry.grid(row=1,column=0)
distance_entry.insert(0,'500')
bearing_entry = tkinter.Entry(canvas, width=label_width) 
bearing_entry.grid(row=2,column=0)
bearing_entry.insert(0,'45')
target_heading_entry = tkinter.Entry(canvas, width=label_width) 
target_heading_entry.grid(row=3,column=0)
target_heading_entry.insert(0,'135')
target_speed_entry = tkinter.Entry(canvas, width=label_width) 
target_speed_entry.grid(row=4,column=0)
target_speed_entry.insert(0,'5.1')
projectile_speed_entry = tkinter.Entry(canvas, width=label_width) 
projectile_speed_entry.grid(row=5,column=0)
projectile_speed_entry.insert(0,'22.6356')
#GUI entry label elements
distance_label = tkinter.Label(canvas,text="distance",font=(font_style, font_size))
distance_label.grid(row=1,column=1,columnspan=5)
bearing_label = tkinter.Label(canvas,text="bearing to target",font=(font_style, font_size))
bearing_label.grid(row=2,column=1,columnspan=5)
target_heading_label = tkinter.Label(canvas,text="target heading",font=(font_style, font_size))
target_heading_label.grid(row=3,column=1,columnspan=5)
target_speed_label = tkinter.Label(canvas,text="target speed",font=(font_style, font_size))
target_speed_label.grid(row=4,column=1,columnspan=5)
target_speed_label = tkinter.Label(canvas,text="projectile speed",font=(font_style, font_size))
target_speed_label.grid(row=5,column=1,columnspan=5)

projectile_lead_angle_label = tkinter.Label(canvas,text="lead:",font=(font_style, font_size))
projectile_lead_angle_label.grid(row=6,column=0,columnspan=1)
projectile_lead_angle_result_label = tkinter.Label(canvas,text="",font=(font_style, font_size))
projectile_lead_angle_result_label.grid(row=6,column=1,columnspan=5)

canvas.pack(fill=tkinter.BOTH, expand=1)
root.geometry(f"{WINDOW_DIMENSIONS.x}x{WINDOW_DIMENSIONS.y}+300+300")
#root.bind('<MouseWheel>', render)#events:Motion,Button,ButtonRelease,MouseWheel,Key
root.after(100,init_lines)
#root.after(200,render)
root.after(200,run)
root.mainloop()




