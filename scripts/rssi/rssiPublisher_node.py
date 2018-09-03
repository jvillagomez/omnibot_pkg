#!/usr/bin/env python
import rospy #import ROS pip package for using ROS library
from geometry_msgs.msg import Point # import our custom ROS msg types
from rssi import RSSI_Scan #Imports the RSSI_scan class from the sibling rssi file
from rssi import RSSI_Localizer

# getNetworks:
    # Description:
        # Parses the 'accessPoints' portion of the rosparams
        # dictionary, and returns a list of network dictioanries,
        # in order. Order specified in yaml params file and.
        # remains persistent.
    # --------------------------------------------------
    # Input:
        # {
        #     'ap1': {
        #         'signalAttenuation': 3, 
        #         'location': {
        #             'y': 7, 
        #             'x': 1
        #         }, 
        #         'reference': {
        #             'distance': 5, 
        #             'signal': 60
        #         }, 
        #         'name': 
        #         'dd-wrt'
        #     }, 
        #     'ap2': {
        #         'name': 'OnePlus3', 
        #         'location': {
        #             'y': 11, 
        #             'x': 3
        #         },
        #         'reference': {
        #             'distance': 4, 
        #             'signal': 40
        #         }, 
        #         'signalAttenuation': 3
        #     },
        # }
    # ------------------------------------------------
    # Returns: (networkNames,accessPoints)
        # networkNames => [
        #   'dd-wrt',
        #   'OnePlus3'
        # ]
        # accessPoints => Ordered list of dictionaries.
        # [
        #     {
        #         'signalAttenuation': 3, 
        #         'location': {
        #             'y': 7, 
        #             'x': 1
        #         }, 
        #         'reference': {
        #             'distance': 5, 
        #             'signal': 60
        #         }, 
        #         'name': 'dd-wrt'
        #     }, 
        #     {
        #         'name': 'OnePlus3', 
        #         'location': {
        #             'y': 11, 
        #             'x': 3
        #         },
        #         'reference': {
        #             'distance': 4, 
        #             'signal': 40
        #         }, 
        #         'signalAttenuation': 3
        #     }
        # ]
def getNetworks(accessPointDict):
    accessPointKeys = accessPointDict.keys()
    accessPointKeys.sort()
    networkNames = []
    accessPoints = []
    for key in accessPointKeys:
        accessPoints.append(accessPointDict[key])
        networkNames.append(accessPointDict[key]['name'])
    
    return networkNames,accessPoints

def rssiPublisher():
    # Single call to the parameter server
    rosParams = rospy.get_param('rssi')
        # {
        #     'accessPoints': {
        #         'ap1': {
        #             'signalAttenuation': 3, 
        #             'location': {
        #                 'y': 7, 
        #                 'x': 1
        #             }, 
        #             'reference': {
        #                 'distance': 5, 
        #                 'signal': 60
        #             }, 
        #             'name': 
        #             'dd-wrt'
        #         }, 
        #         'ap2': {
        #             'name': 'OnePlus3', 
        #             'location': {
        #                 'y': 11, 
        #                 'x': 3
        #             },
        #             'reference': {
        #                 'distance': 4, 
        #                 'signal': 40
        #             }, 
        #             'signalAttenuation': 3
        #         },
        #     }
        #     'networkInterface': 'wlp1s0'
        # }

    pub = rospy.Publisher('rssiLocalization_topic', Point, queue_size=10)
    # Get parameters from our ROS server
    networkInterface = rosParams['networkInterface']
    ssids,networks = getNetworks(rosParams['accessPoints'])
    # Initialize node
    rospy.init_node('rssiPublisher_node')
    # Create instance of RSSI module.
    # This makes our 'getAccessPoints' function available to us.
    rssi = RSSI_Scan(networkInterface)
    localizer = RSSI_Localizer(networks)
    # This keeps the node running until the script is shut down manually.
    # node will keep cycling at the frequncy set above.
    while not rospy.is_shutdown(): 
        # Returns an array of all access-points of interest (if they are in range).
        # Process takes about 1.5-4 seconds to finish.
        ap_info = rssi.getAPinfo(networks=ssids, sudo=True)
        if not ap_info:
            rospy.loginfo('Nodes not found')
            pub.publish(-1,-1,-1)
            continue
        
        rssi_values = [ap['signal'] for ap in ap_info]
        
        distances = localizer.getDistancesForAllAPs(rssi_values)
        distances = [distance['distance'] for distance in distances]
        
        position = localizer.getNodePosition(rssi_values)
        for ap in ap_info:
            rospy.loginfo(ap)
        rospy.loginfo(position)
        pub.publish(position[0],position[1],0)
            
if __name__ == '__main__':
    try:
        rssiPublisher()
    except rospy.ROSInterruptException:
        pass
