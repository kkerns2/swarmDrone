# **Map Merge Tool**

## Instructions to Run:
```bash
$ cd build/
$ cmake ..
$ make
```

./DisplayImage inputfile1 inputfile2 outputfile

```bash
$ ./DisplayImage figure/map2.1.pgm figure/map2.2.pgm result/output.pgm
```

To do stop, please ctrl+C

```bash
rotation: -0.0277336
translation (x,y): 0.11894, -0.956272
matrix: [1.001368923533066, -0.0004847044592732864, 0.1189402785106077;
 0.0004847044592732864, 1.001368923533066, -0.9562716407693578]
imwrite_('debug.pgm'): can't write data: OpenCV(4.1.1) /home/nvidia/host/build_opencv/nv_opencv/modules/imgcodecs/src/grfmt_pxm.cpp:427: error: (-5:Bad argument) Portable bitmap(.pgm) expects gray image in function 'write'

rotation: -0.0277336
translation (x,y): 0.11894, -0.956272
matrix: [1.001368923533066, -0.0004847044592732864, 0.1189402785106077;
 0.0004847044592732864, 1.001368923533066, -0.9562716407693578]
imwrite_('debug.pgm'): can't write data: OpenCV(4.1.1) /home/nvidia/host/build_opencv/nv_opencv/modules/imgcodecs/src/grfmt_pxm.cpp:427: error: (-5:Bad argument) Portable bitmap(.pgm) expects gray image in function 'write'

^C
```
## Result
![117106967-c6290a00-adbb-11eb-9c0d-34a4b076ee35](https://user-images.githubusercontent.com/52307432/117110067-929cae80-adc0-11eb-83cb-d5ccc30ede20.png)

![117107003-d80aad00-adbb-11eb-9d4c-1b53bc453d72](https://user-images.githubusercontent.com/52307432/117110212-beb82f80-adc0-11eb-8c70-e49bf1c09bd2.png)

## Create yaml file
The map is managed by a pair of settings in a YAML file and an image file of the map itself.  The darker the gray in the image file, the more obstacles are treated as obstacles.  Its threshold is determined by the YAML file.  The image file can be color or gray, it doesn't matter.  As for the ROS message, occupancy is represented by a number between 0 and 100 (0 being free and 100 being a complete wall). Also, unobserved areas are set to -1.  The format of the image file can be anything that can be read by SDL.  

The YAML file looks like the following. 

-------------------------------------------
image: Image file (relative or absolute path from YAML)  
resolution: [meters/pixel]  
origin: Coordinates of the lower left edge of the map (x,y,θ)
occupied_thresh: Anything larger than this number is considered an obstacle  
free_thresh: Areas smaller than this number are considered free space.    
negate: Whether to invert black and white or not

Example
```yaml
image: output.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
-------------------------------------------

## Points to note
The two maps you want to integrate need to have some degree of overfitting to be able to integrate well.
