#!/bin/bash


g++ planner.cpp -o planner.out -g


echo "----------RRT (0)-------------"

./planner.out map2.txt 5 0.652432,1.93125,0.860195,4.05817,0.735575 0.493918,1.56935,3.36496,2.4795,4.096 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt1.gif
./planner.out map2.txt 5 0.717638,1.67035,1.22259,5.82314,5.04121 0.122001,1.79388,4.58133,1.34459,1.33384 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt2.gif
./planner.out map2.txt 5 1.87432,1.6701,0.656577,0.172465,3.01469 0.208275,2.01372,5.93779,5.6466,2.20514 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt3.gif
./planner.out map2.txt 5 1.60309,1.69365,6.21852,0.465455,4.10384 1.72631,0.462405,0.618807,3.92077,4.65614 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt4.gif
./planner.out map2.txt 5 1.33401,5.46338,2.45872,2.01888,2.62059 0.584495,1.91973,1.35622,5.7559,2.90894 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt5.gif
./planner.out map2.txt 5 1.80205,0.262999,4.96894,6.28258,3.10544 1.82529,0.5846,3.81504,2.33055,4.44703 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt6.gif
./planner.out map2.txt 5 1.61756,2.83166,1.74987,0.574301,1.53551 1.51102,5.90349,5.69665,1.34955,4.26161 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt7.gif
./planner.out map2.txt 5 1.75223,5.467,2.34361,2.06755,4.6425 0.195863,2.87109,1.79959,3.2159,4.37857 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt8.gif
./planner.out map2.txt 5 0.0975708,0.759666,2.86549,2.75685,2.45442 0.852278,1.71209,1.58936,0.203251,4.1895 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt9.gif
./planner.out map2.txt 5 0.153279,2.9266,1.56858,5.49196,1.42259 1.43449,2.62251,1.88224,5.69503,5.45901 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt10.gif
./planner.out map2.txt 5 1.56185,2.42496,1.23919,4.18571,1.63079 0.0249774,2.24864,1.93738,2.44227,3.66393 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt11.gif
./planner.out map2.txt 5 1.18887,2.17637,0.392124,4.98453,4.14347 1.57269,5.78496,1.90018,3.82518,1.01714 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt12.gif
./planner.out map2.txt 5 1.34491,2.792,0.0394511,5.3128,0.848485 1.13205,5.87905,2.46231,5.49613,3.34452 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt13.gif
./planner.out map2.txt 5 1.56379,0.421933,1.84615,2.61139,1.52814 1.07284,0.301974,4.44542,1.33387,1.60018 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt14.gif
./planner.out map2.txt 5 1.3294,1.80765,5.05074,5.77482,3.14153 1.74287,0.880885,3.19568,2.8383,1.46608 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt15.gif
./planner.out map2.txt 5 1.05227,2.0794,5.43266,4.06564,6.09198 1.07726,5.60969,3.05913,0.0874643,2.05544 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt16.gif
./planner.out map2.txt 5 1.79939,6.01357,2.40583,2.79022,2.72471 0.729222,2.22121,2.70895,2.47181,5.66707 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt17.gif
./planner.out map2.txt 5 0.504473,2.48276,2.55424,1.86238,4.96584 0.783771,1.60667,2.82915,0.95083,1.49328 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt18.gif
./planner.out map2.txt 5 1.41977,5.4001,2.20373,6.0465,2.42001 0.155651,2.09209,1.16245,3.43439,2.63388 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt19.gif
./planner.out map2.txt 5 0.506921,2.50711,0.754732,0.896646,0.479126 1.58599,1.16138,2.83523,2.69196,1.20672 0 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt20.gif




echo "----------RRT-C (1)-------------"
./planner.out map2.txt 5 0.652432,1.93125,0.860195,4.05817,0.735575 0.493918,1.56935,3.36496,2.4795,4.096 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c1.gif
./planner.out map2.txt 5 0.717638,1.67035,1.22259,5.82314,5.04121 0.122001,1.79388,4.58133,1.34459,1.33384 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c2.gif
./planner.out map2.txt 5 1.87432,1.6701,0.656577,0.172465,3.01469 0.208275,2.01372,5.93779,5.6466,2.20514 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c3.gif
./planner.out map2.txt 5 1.60309,1.69365,6.21852,0.465455,4.10384 1.72631,0.462405,0.618807,3.92077,4.65614 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c4.gif
./planner.out map2.txt 5 1.33401,5.46338,2.45872,2.01888,2.62059 0.584495,1.91973,1.35622,5.7559,2.90894 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c5.gif
./planner.out map2.txt 5 1.80205,0.262999,4.96894,6.28258,3.10544 1.82529,0.5846,3.81504,2.33055,4.44703 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c6.gif
./planner.out map2.txt 5 1.61756,2.83166,1.74987,0.574301,1.53551 1.51102,5.90349,5.69665,1.34955,4.26161 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c7.gif
./planner.out map2.txt 5 1.75223,5.467,2.34361,2.06755,4.6425 0.195863,2.87109,1.79959,3.2159,4.37857 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c8.gif
./planner.out map2.txt 5 0.0975708,0.759666,2.86549,2.75685,2.45442 0.852278,1.71209,1.58936,0.203251,4.1895 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c9.gif
./planner.out map2.txt 5 0.153279,2.9266,1.56858,5.49196,1.42259 1.43449,2.62251,1.88224,5.69503,5.45901 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c10.gif
./planner.out map2.txt 5 1.56185,2.42496,1.23919,4.18571,1.63079 0.0249774,2.24864,1.93738,2.44227,3.66393 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c11.gif
./planner.out map2.txt 5 1.18887,2.17637,0.392124,4.98453,4.14347 1.57269,5.78496,1.90018,3.82518,1.01714 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c12.gif
./planner.out map2.txt 5 1.34491,2.792,0.0394511,5.3128,0.848485 1.13205,5.87905,2.46231,5.49613,3.34452 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c13.gif
./planner.out map2.txt 5 1.56379,0.421933,1.84615,2.61139,1.52814 1.07284,0.301974,4.44542,1.33387,1.60018 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c14.gif
./planner.out map2.txt 5 1.3294,1.80765,5.05074,5.77482,3.14153 1.74287,0.880885,3.19568,2.8383,1.46608 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c15.gif
./planner.out map2.txt 5 1.05227,2.0794,5.43266,4.06564,6.09198 1.07726,5.60969,3.05913,0.0874643,2.05544 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c16.gif
./planner.out map2.txt 5 1.79939,6.01357,2.40583,2.79022,2.72471 0.729222,2.22121,2.70895,2.47181,5.66707 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c17.gif
./planner.out map2.txt 5 0.504473,2.48276,2.55424,1.86238,4.96584 0.783771,1.60667,2.82915,0.95083,1.49328 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c18.gif
./planner.out map2.txt 5 1.41977,5.4001,2.20373,6.0465,2.42001 0.155651,2.09209,1.16245,3.43439,2.63388 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c19.gif
./planner.out map2.txt 5 0.506921,2.50711,0.754732,0.896646,0.479126 1.58599,1.16138,2.83523,2.69196,1.20672 1 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-c20.gif

echo "----------RRT* (2)-------------"
./planner.out map2.txt 5 0.652432,1.93125,0.860195,4.05817,0.735575 0.493918,1.56935,3.36496,2.4795,4.096 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s1.gif
./planner.out map2.txt 5 0.717638,1.67035,1.22259,5.82314,5.04121 0.122001,1.79388,4.58133,1.34459,1.33384 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s2.gif
./planner.out map2.txt 5 1.87432,1.6701,0.656577,0.172465,3.01469 0.208275,2.01372,5.93779,5.6466,2.20514 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s3.gif
./planner.out map2.txt 5 1.60309,1.69365,6.21852,0.465455,4.10384 1.72631,0.462405,0.618807,3.92077,4.65614 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s4.gif
./planner.out map2.txt 5 1.33401,5.46338,2.45872,2.01888,2.62059 0.584495,1.91973,1.35622,5.7559,2.90894 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s5.gif
./planner.out map2.txt 5 1.80205,0.262999,4.96894,6.28258,3.10544 1.82529,0.5846,3.81504,2.33055,4.44703 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s6.gif
./planner.out map2.txt 5 1.61756,2.83166,1.74987,0.574301,1.53551 1.51102,5.90349,5.69665,1.34955,4.26161 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s7.gif
./planner.out map2.txt 5 1.75223,5.467,2.34361,2.06755,4.6425 0.195863,2.87109,1.79959,3.2159,4.37857 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s8.gif
./planner.out map2.txt 5 0.0975708,0.759666,2.86549,2.75685,2.45442 0.852278,1.71209,1.58936,0.203251,4.1895 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s9.gif
./planner.out map2.txt 5 0.153279,2.9266,1.56858,5.49196,1.42259 1.43449,2.62251,1.88224,5.69503,5.45901 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s10.gif
./planner.out map2.txt 5 1.56185,2.42496,1.23919,4.18571,1.63079 0.0249774,2.24864,1.93738,2.44227,3.66393 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s11.gif
./planner.out map2.txt 5 1.18887,2.17637,0.392124,4.98453,4.14347 1.57269,5.78496,1.90018,3.82518,1.01714 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s12.gif
./planner.out map2.txt 5 1.34491,2.792,0.0394511,5.3128,0.848485 1.13205,5.87905,2.46231,5.49613,3.34452 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s13.gif
./planner.out map2.txt 5 1.56379,0.421933,1.84615,2.61139,1.52814 1.07284,0.301974,4.44542,1.33387,1.60018 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s14.gif
./planner.out map2.txt 5 1.3294,1.80765,5.05074,5.77482,3.14153 1.74287,0.880885,3.19568,2.8383,1.46608 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s15.gif
./planner.out map2.txt 5 1.05227,2.0794,5.43266,4.06564,6.09198 1.07726,5.60969,3.05913,0.0874643,2.05544 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s16.gif
./planner.out map2.txt 5 1.79939,6.01357,2.40583,2.79022,2.72471 0.729222,2.22121,2.70895,2.47181,5.66707 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s17.gif
./planner.out map2.txt 5 0.504473,2.48276,2.55424,1.86238,4.96584 0.783771,1.60667,2.82915,0.95083,1.49328 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s18.gif
./planner.out map2.txt 5 1.41977,5.4001,2.20373,6.0465,2.42001 0.155651,2.09209,1.16245,3.43439,2.63388 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s19.gif
./planner.out map2.txt 5 0.506921,2.50711,0.754732,0.896646,0.479126 1.58599,1.16138,2.83523,2.69196,1.20672 2 myOutput.txt
python3 visualizer.py myOutput.txt --gifFilepath rrt-s20.gif