from Image_Tiler import Image_Tiler
import rospy


def Main():
    tiler = Image_Tiler(
        read_path="/home/mcp/catkin_ws/src/swarm/maps/urban_small.pgm",
        write_path="/home/mcp/catkin_ws/src/swarm/waypoints/",
        numTilesY=50,
        numTilesX=50,
        numBots=4,
        minMaxer=False
    )

    tiler.IMGSLICE()
    tiler.SHOWIMG()

Main()