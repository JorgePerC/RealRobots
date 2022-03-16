
import xml.dom.minidom
def getBoxCoordinateFromXML(fileName):
    # After we've included the corresponding library (and installed it) 
    # We call the following code to get the poses (coordinates and orientation)
    # For the boxes in the /xarm_example1_table.world
    positions = []
    xmlFile = xml.dom.minidom.parse(fileName)
    boxes = xmlFile.getElementsByTagName("model")
    
    for box in boxes:
        sid = box.getAttribute("name")
        print(sid)
        print(box.lastChild.data) #  ¯\_(ツ)_/¯    
    
    return positions

fileName = "/home/jorgepc/Documents/RealRobots/Reto/challenge/xarm_challenge/src/xarm_ros/xarm_gazebo/worlds/xarm_pickplace_test.world"
print("\t Going to ", fileName)
objectives = getBoxCoordinateFromXML(fileName)