# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import time

# ----------------------------------------------------------------
#Result:
#The Verlet method is very precise and has almost no different with the analytical soltion (when dt is not that large)
#In comparison, the Euler method has very significant error acculation when dt is larger than 0.01, the critical value
#The result from verlet method is relatively reliable before dt is larger than 1.1, the critical value. The result becomes absolutely non sense when dt is larger than 2

start=time.perf_counter()
# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt=0.1
t_array = np.arange(0, t_max, dt)

#The analytical solution
num_points=1000
z=np.linspace(0, 100, num=num_points) 
f=np.cos(z) #s
g=np.sin(z) #v


# initialise empty lists to record trajectories
x_list = []
v_list = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -k * x / m
    x = x + dt * v
    v = v + dt * a

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)

end=time.perf_counter()
print(end-start)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.plot(z, f, label='analytical x (m)')
plt.plot(z, g,label='analytical v (m/s)')
plt.legend()
#plt.show()



#Verlet Algorithm
start=time.perf_counter()
m = 1
k = 1
x_2 = 0 # past two, the initial position
v_2 = 1 #Initial velocity

x_1=x_2 + dt * v_2 #past one, the second initial position

x_list_2 = []
v_list_2 = []

x_list_2.append(x_2)
x_list_2.append(x_1)

v_1=v_2+dt*(-k * x_2 / m)
v_list_2.append(v_2)
v_list_2.append(v_1)

for t in t_array[2:]:
    a = -k * x_1 / m
    x=2*x_1-x_2+(dt*dt)*a

    x_list_2.append(x)

    v=(x-x_1)/(dt)
    v_list_2.append(v)

    x_2=x_1
    x_1=x

x_array_2 = np.array(x_list_2)
v_array_2 = np.array(v_list_2)
end=time.perf_counter()
print(end-start)
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(z, f, label='analytical x (m)')
plt.plot(z, g,label='analytical v (m/s)')
plt.plot(t_array, x_array_2, label='x (m)')
plt.plot(t_array, v_array_2, label='v (m/s)')
plt.legend()
#plt.show()


#------------------------
#Assignment 2

x_array=np.array([],dtype=np.float64)
r_list=[]
v_list=[]

radius=3.386*10**6 


def modulus(array): #only applies for 3D
    r=np.sqrt(abs(array[0]**2+array[1]**2+array[2]**2)) 
    return r

def verlet(pos,vel,t_max,orbital=False):
    t_array=np.array([],dtype=np.float64)
    r_list=[]
    pos_list=[]

    dt=1.6
    t_array = np.arange(0, t_max, dt)
    #r=np.sqrt(abs(pos[0]**2+pos[1]**2+pos[2]**2)) 
    r=modulus(pos)
    a=(force_const)/(r**2) #in the direction (-x,-y,-z) /|r|
    acceleration=np.array([a*pos[0]/(r),a*pos[1]/(r),a*pos[2]/(r)],dtype=np.float64)

    pos_1=pos + dt * vel
    vel_1=vel+dt*acceleration
    r_list.append((r-radius))
    r_list.append(modulus(pos_1)-radius)
    pos_list.append(pos)
    pos_list.append(pos_1)
    i=0

    v_list = []
    v_list.append(modulus(vel))
    v_list.append(modulus(vel_1))

    for t in t_array[2:]:
        
        a=force_const/(r**2)
        position=2*pos_1-pos+(dt*dt)*(np.array([(a*(pos_1[0]))/(r),(a*(pos_1[1]))/(r),(a*(pos_1[2]))/(r)],dtype=np.float64))
        #print(a) This reveals that the ultimate a is about3.72m/s^2. By wikepedia, this is correct
        r=modulus(position)
        pos_list.append(position[0:-1])
        r_list.append(r-radius)

        velocity=(position-pos)/(2*dt)
        v_list.append(modulus(velocity))

        pos=pos_1
        pos_1=position

        i+=1
        if r_list[-1]>=0:
            pass
        else:
            r_list=r_list[:-1]
            t_array=t_array[:i+1]
            v_list=v_list[:-1]
            break

    if orbital==False:#This plots v-t and r-t diagram
        r_array = np.array(r_list)
        v_array=np.array(v_list)
        plt.figure(1)
        plt.clf()
        plt.xlabel('time (s)')
        plt.grid()
        plt.plot(t_array,r_array, label='x (m)')
        plt.legend()
        plt.show()
        plt.plot(t_array,v_array,label='v (m/s)')
        plt.show()

    else: #This plots in the trajectory plane
        x_axis=list(pos_list[i][0] for i in range(len(pos_list)))
        y_axis=list(pos_list[i][1] for i in range(len(pos_list)))
        plt.scatter(0,0)
        plt.axis('equal')
        plt.plot(x_axis,y_axis)
        plt.show()



force_const =-6.67*10**(-11)*6.42*10**(23)#-GM
m=1

#Test Cases
# 
# Case 1 Simply descent
x=y=z=3690000 
vx=vy=vz=0 #5030m/s is the excape speed from the surface
position=np.array([x,y,z],dtype=np.float64)
velocity=np.array([vx,vy,vz],dtype=np.float64)
verlet(position,velocity,200000) 

#Case 2 Circular orbit 
x=10000+radius
y=z=0
vx=vz=0
vy=np.sqrt(x*3.713)
position=np.array([x,y,z],dtype=np.float64)
velocity=np.array([vx,vy,vz],dtype=np.float64)
verlet(position,velocity,200000,orbital=True)


# Case 3 Elipse 
x=0
y=-(radius+10000)
z=0
dt=0.1
vx=4000
vy=vz=0
position=np.array([x,y,z],dtype=np.float64)
velocity=np.array([vx,vy,vz],dtype=np.float64)
verlet(position,velocity,100000,orbital=True) 

# Case 4 Hyperbolic
x=y=z=3390000
vx=1.22*np.sqrt((-force_const)/(3.39*10**6  * np.sqrt(6))) #5030m/s is the excape speed from the surface
vy=-1.22*np.sqrt((-force_const)/(3.39*10**6  * np.sqrt(6)))
vz=0
position=np.array([x,y,z],dtype=np.float64)
velocity=np.array([vx,vy,vz],dtype=np.float64)
verlet(position,velocity,100000,orbital=True)


#Additional check Case 1
x=0
y=-(radius+10000)
z=0
dt=0.1
vx=vy=vz=0
position=np.array([x,y,z],dtype=np.float64)
velocity=np.array([vx,vy,vz],dtype=np.float64)
verlet(position,velocity,100000,orbital=False) #Result: 75s, 260m/s accurate enough consider that there's no drag


