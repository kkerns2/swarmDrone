from mimetypes import init
import cv2
from PIL import Image
import numpy as np
from tqdm import tqdm

class Image_Tiler():
    

    def __init__(self, read_path: str, write_path: str, numTilesY: int, numTilesX: int, 
    numBots: int, minMaxer: bool) -> None:
        """
        Takes a saved .pgm map generated after running the saver and map-merge
        commands and processes to generate a set of waypoints throughout the map.
        Waypoints are also split amongst all present robots by rows.  
        
        Waypoints are also reversed every other row so once a robot completes navigating a row,
        it takes the robot down one row, then backtracks across the map rather than
        taking it back to the opposite side of the map, then beginning the next row.

        NOTE: Waypoints will not be generated for some robots if:

            - Too many robots are present in the search space
            - The number of tiles along each axis is to few
        
        Variables:

        - read_path  -> String of the path to the map.pgm file being read
        - write_path -> String of the path to write waypoints out to
        - numTilesY  -> Number of tiles along the Y axis of the image
        - numTilesX  -> Number of tiles along the X axis of the image
        - numBots    -> Number of robots within the swarm
        - minMaxer   -> Optional min/maxing of input image contrast for improved accuracy

            - NOTE: Using minMaxer significantly increases processing time.
        """
        self.image_path = read_path
        self.image_raw = Image.open(self.image_path)

        self.image_raw.save("/home/mcp/catkin_ws/src/swarm/maps/converted_map.png")
        self.image_raw.close()

        self.image_png = cv2.imread("/home/mcp/catkin_ws/src/swarm/maps/converted_map.png")
        

        # Convert the PNG image to RGB format for more accurate processing later.
        self.converted_image = cv2.cvtColor(self.image_png, cv2.COLOR_BGR2RGB)

        self.numTilesY = numTilesY # Number of tiles along the Y axis of the image.
        self.numTilesX = numTilesX # Number of tiles along the X axis of the image.
        self.numBots = numBots # Number of robots within the swarm.
        self.minMaxer = minMaxer # Boolean to triger min/max the contrast of the image
    

    def SHOWIMG(self):

        self.converted_image = cv2.resize(self.converted_image, (1024, 1024))
        cv2.imshow('map', self.converted_image)
        cv2.waitKey(5000)

    def IMGSLICE(self):
        imageHeight = self.converted_image.shape[0]
        imageWidth = self.converted_image.shape[1]
        waypoints = list()
        yRange = 0
        xRange = 0
        rowCount = 0
        droneCount = 1
        point_list = list()

        # Create waypoint files for each robot
        files = {}
        for i in range(1, self.numBots):
            files["file{0}".format(i)] = open("/home/mcp/catkin_ws/src/swarm/waypoints/waypoints_d%i.txt" %i, "w")

        current_file = files["file{0}".format(droneCount)]

        if self.minMaxer == True:
            # Increase complete contrast
            for i in tqdm(range(0, imageHeight)):
                for j in range(0, imageWidth):
                    if np.mean(self.converted_image[i][j]) < 254:
                        self.converted_image[i][j] = 0
                    else:
                        self.converted_image[i][j] = 255
        
        for y in tqdm(range(0, imageHeight, self.numTilesY)):
            flag = 0
            for x in range(0, imageWidth, self.numTilesX):
                yRange = y + self.numTilesY
                xRange = x + self.numTilesX

                tile = self.converted_image[y:yRange, x:xRange]

                if np.mean(tile) > 250:
                    tile_height = tile.shape[0]
                    tile_width = tile.shape[1]

                    # Find the tile center and convert the stored coordinates to Rviz coordinates
                    center = ((x+tile_width//2), (y+tile_height//2))
                    center_meters = [np.floor(center[0] * 0.1 - 150.0), np.floor(-1 * (center[1] * 0.1 - 150.0))]

                    point_list.append(center_meters)
                    flag = 1

                    cv2.putText(self.converted_image, str(center_meters), (x, y+tile_height), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 200, 0))
                    cv2.rectangle(self.converted_image, (x, y), (xRange, yRange), (160, 32, 240))
                    cv2.circle(self.converted_image, center, 5, (160, 32, 240))
                
                # Visualize the plotted waypoint
                cv2.imwrite("~/catkin_ws/src/swarm/maps/converted_map.png", tile)
            
            # Reverse the points of every other row.
            if rowCount % 2 != 0:
                point_list.reverse()
            
            # Append the waypoints list
            for i in range(len(point_list)):
                waypoints.append(point_list[i])
            
            point_list.clear()

            if flag == 1:
                rowCount += 1

                if rowCount == 3 or droneCount == self.numBots-1:
                    # Write waypoints to the current file
                    for i in range(len(waypoints)):
                        current_file.write(str(waypoints[i][0]) + " " + str(waypoints[i][1]))
                        current_file.write("\n")
                    droneCount += 1

                    if droneCount <= len(files):
                        cv2.line(self.image_png, (0, y + tile_height), (imageWidth, y + tile_height), (0, 0, 0), 4)
                        current_file = files["file{0}".format(droneCount)]
                        waypoints.clear()
                        rowCount = 0
        
        # Close all files
        for i in range(1, self.numBots):
            files["file{0}".format(i)].close()