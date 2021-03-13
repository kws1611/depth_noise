
import airsim
import cv2
import numpy as np

client = airsim.VehicleClient()
client.confirmConnection()

#set object ID for Ground_6
# you can change segmentation ID by changing name of mesh and ID number
airsim.wait_key('Press any key to set all object IDs to 0')
found = client.simSetSegmentationObjectID("Ground_6", 0, True);
print("Done: %r" % (found))



#example of changing mesh(SKY_example) as ID number 10
'''
airsim.wait_key('Press any key to set all object IDs to 0')
found = client.simSetSegmentationObjectID("SKY_example", 10, True);
print("Done: %r" % (found))
'''
 
