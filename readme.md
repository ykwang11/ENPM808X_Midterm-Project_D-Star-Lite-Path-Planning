# D* Lite Path Planning
---
[![Build Status](https://travis-ci.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning.svg?branch=master)](https://travis-ci.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning)
[![Coverage Status](https://coveralls.io/repos/github/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/badge.svg?branch=master)](https://coveralls.io/github/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning?branch=master)
---
## Overview

Self-driving car has been an important industrial product recently to save millions of lives from car accidents. The field of domain involves three main aspects: Perceptron, Path Planning, and Control. The object of this project is to implement of `D* Lite search in 2D map`.

`A star searching algorithm (A*)`, a search algorithm using its heuristic knowledge in path planning, has been proved to be a successful in static well-known environment. However, A* does not able to with or environmental changes such as disturbances introduced by traffic lights, pedestrians or surrounding cars, which are common on the street. To address this issue, Sven Koenig proposed Dynamic A* algorithm (D*) and its lightweight version, D* Lite algorithm, in 1994 and 2002. 

`D* Lite algorithm` is an incremental heuristics algorithm. In other words, the robot does heuristic search in the beginning like A*. The different is that D* Lite assumes environmental changes are relatively small compared to the whole map. On the top of that, once the robot detects the change during the search, it can reuse the information from the previous search to focus on changes. Incrementally repairing an existing solution is much faster than utilizing A* to re-planning shortest path from scratch.

The design process follows the iterative process of a solo programmer (SIP) and Test Driving Development (TDD). The project is built on C++ 11/14 framework following Google C++ Style guide. Tools such as “cpplint validation” and “cppcheck” will be used to ensure the quality of codes. Methods were verified by unit testing with Google Test framework.

# Eample:  
At first, the terminal displays g values and rhs values after computing the shortest path based on A* and the visualization.  
g-values: estamates distance to the goal  
rhs-values: one step lookahead values based on the g values
![image](https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/results/visual_demo/ENPM808X-1.png)

Secondly, when the robots detects the hidden obstacle (`?` in the visualization)
![image](https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/results/visual_demo/ENPM808X-2.png)

The robots realizes it is actually an obstacle (`x` in the visualization).   
So, it re-computes the shortest path only with thoses nodes matter.  
That is an incremental search.  
![image](https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/results/visual_demo/ENPM808X-3.png)

At last, the robot reaches goal with the minimum cost.  
![image](https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning/blob/master/results/visual_demo/ENPM808X-4.png)

---

---
## Licence
* MIT Licence  

```
Copyright (c) 2018 Yu-Kai Wang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---
## Install via Command-line  

Clone from the repository, create build folder and build program.  

* Build and make:
```
git clone --recursive https://github.com/ykwang11/ENPM808X_Midterm-Project_D-Star-Lite-Path-Planning.git  
cd <path to repository>  
mkdir build  
cd build  
cmake ..  
make  
```
* Run tests:  
```  
cd build  
./test/cpp-test  
```  
* Run program:   
```  
cd build  
./app/shell-app  
```  
* Run Doxygen:  
```  
doxygen ./Doxygen 
```

---
## SIP Process
[SIP process is recorded in product backlog, iteration backlog, and work log.](https://drive.google.com/drive/folders/1RUn3eqfVhPALQS88JPey561u3MOfQhIE?usp=sharing)

---
## Reference
[S. Koenig and M. Likhachev. D* Lite. In Proceedings of the AAAI Conference of Artificial Intelligence (AAAI), 476-483, 2002.](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)

