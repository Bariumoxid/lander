#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>

using namespace std;

//Findings
//Windows 11, Intel i7-9750H 2.60GHz
/* (ALL run without debug mode on)
For the Euler simulation, dt=0.00001 
    For C++ x64
        Under Debug mode (no optimization): 3.31s 

        Under Release mode (with optimization):0.309s

    For C++ x86
        Under Debug mode (no optimization): 8.5 s

        Under Release mode (with optimization): 0.38s

    For python: 7.69s, way slower than C++ if under x64

 Release mode, i.e. the compilation with optimization is much faster than the debug mode, without optimization
  The excution took almost the same amount of time under release mode, no matter x64 or x86
 The python code is way solwer (2-20 times) slower than C++ x64

 For Verlet simulation, dt=0.00001 
    For C++ x64
        Under Debug mode (no optimization): 3.8s

        Under Release mode (with optimization):3.617s 

    #No big difference due to optimization

    For C++ x86
        Under Debug mode (no optimization): 9.16s

        Under Release mode (with optimization):0.327s #Dont know why releause under x64 is so slow compare to x86

    For python: 9.86s
    
    The results for Verlet is similar to the results obtained above. It is evident that the Verlet method is slightly slower than
    Euler method but it has higher accuracy.
*/

int main() {
  long i = 10000000L;
  clock_t start, end;
  start = clock();
  // declare variables
  double m, k, x, v, t_max, dt, t, a;
  vector<double> t_list, x_list, v_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;

  // simulation time and timestep
  t_max = 100;
  dt = 0.00001;

  // Euler integration
 // for (t = 0; t <= t_max; t = t + dt) 

      // append current state to trajectories
      //t_list.push_back(t);
      //x_list.push_back(x);
      //v_list.push_back(v);

      // calculate new position and velocity
  //    a = -k * x / m;
  //    x = x + dt * v;
  //    v = v + dt * a;
  //}
  //end = clock();
  //printf("Time: %f\n", (double)(end - start) / CLK_TCK);


//Verlet 
  double x_2, x_1,v_2,v_1;
  x_2 = x;
  x_1 = x_2 + dt * v;
  v_2 = v;
  v_1 = v_2 + dt * (-k * x_2 / m);

  x_list.push_back(x_2);
  x_list.push_back(x_1);
  v_list.push_back(v_2);
  v_list.push_back(v_1);
  t_list.push_back(0);
  t_list.push_back(dt);

  for (t = 2*dt; t <= t_max; t = t + dt) {
      t_list.push_back(t);
      a = -k * x_1 / m;
      x = 2 * x_1 - x_2 + (dt * dt) * a;

      x_list.push_back(x);

      v = (x - x_1) / (dt);
      v_list.push_back(v);

      x_2 = x_1;
      x_1 = x;
     }
  end = clock();
  printf("Time: %f\n", (double)(end - start) / CLK_TCK);
  // Write the trajectories to file
  ofstream fout;
  fout.open("trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }

  /* The file can be loaded and visualised in Python as follows:

  import numpy as np
  import matplotlib.pyplot as plt
  results = np.loadtxt('trajectories.txt')
  plt.figure(1)
  plt.clf()
  plt.xlabel('time (s)')
  plt.grid()
  plt.plot(results[:, 0], results[:, 1], label='x (m)')
  plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
  plt.legend()
  plt.show()

  */
}
