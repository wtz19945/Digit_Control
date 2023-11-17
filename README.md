
# A Digit standing controller implementation

## Note
Compiling the code requires an updated of version of gcc (version 7.5 at least) and Cmake (version 3.14 as least). If you have issues, please check the following link:

CMAKE: https://gist.github.com/bmegli/4049b7394f9cfa016c24ed67e5041930

GCC  : https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu

## Library Requirements
To run the code, you need to install the following libraries 

OSQP-Eigen Github       : https://github.com/robotology/osqp-eigen

## Start Simulator
To run the Agility simulator, run the following command
- chmod +x ar-control
- ./ar-control ./examples/lowlevelapi_example.toml

This will start a webpage at localhost:8080. Open this link with your browser. You can read the detailed Digit documentation or simulate Digit robot in this webpage.

To enable controller via low-level-api operation mode, check the following link http://localhost:8080/doc/software/lowlevelapi.html

## Compile the code
To compile the code, run:
- cmake ./
- make standing_control
- ./standing_control

When the operation mode is set to low-level-api, you should see optimization status from the terminal and digit starts moving.

Note that you need to modify the directories depending on where you store the files and delete CMakeCache.txt to have cmake running
