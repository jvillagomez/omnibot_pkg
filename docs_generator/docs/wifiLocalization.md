# Wifi Localization

RSSI-based self-loclaization is utilized as a substitute for Bluetooth and/or GPS localization.
Our module was designed entirely around Xiuyan Zhu's and Yuan Feng's 'RSSI-based Algorithm for Indoor Localization' 
paper, published here: https://file.scirp.org/pdf/CN_2013071010352139.pdf

Using the wifi signal detected from three separate access points, we can use the log-model (as described in the paper) 
to estimate our distance from each access point. With a distance estimated fomr each node, we use the trilateral-localization
algortihm to estimate a location.

Note: The location of each node must not be fixed, but must be specified.


Log Model
CS = Current Signal from node x
RS = Reference Signal for node x
CD = Current Distance from node x
RD = Reference Distance for node x
n = Signal Attenuation Factor
log = log w/base-10

[CS] = [RS]-[10*nlog(CD/RD)]

Solving for CD:
[Beta] = [(RS-CS)/(10*n)]
[CD] = [10^Beta]*[RD]

For more information on the python rssi module, visit: 
https://github.com/jvillagomez/rssi_module


## RSSI Module
### Scan Class

``` python
#!/usr/bin/env python
# RSSI_Scan
    # Use:
        # from rssi import RSSI_Scan
        # rssi_scan_instance = RSSI_Scan('network_interface_name) 
    # -------------------------------------------------------
    # Description:
        # Allows a user to query all available accesspoints available.
        # User has the option of define a specific set of access 
        # points to query.
    # -------------------------------------------------------
    # Input: interface name
        # [ie. network interface names: wlp1s0m, docker0, wlan0] 
class RSSI_Scan(object):
    # Allows us to declare a network interface externally.
    def __init__(self, interface):
        self.interface = interface

    # getRawNetworkScan
        # Description:
            # Runs the Ubuntu command 'iwlist' to scan for available networks.
            # Returns the raw console window output (unparsed).
        # ----------------------------------------------------------------
        # Input: (optional) 
            #   sudo: bool; defaults to false. False will not refresh the 
            #         network interface upon query. Sudo=true will require 
            #         the user will need to enter a sudo password at runtime.
        # ----------------------------------------------------------------
        # Returns: Raw terminal output
            # {
            #     'output':'''wlp1s0    Scan completed :
            #   Cell 01 - Address: A0:3D:6F:26:77:8E
            #             Channel:144
            #             Frequency:5.72 GHz
            #             Quality=43/70  Signal level=-67 dBm  
            #             Encryption key:on
            #             ESSID:"ucrwpa"
            #             Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            #             Mode:Master
            #   Cell 02 - Address: A0:3D:6F:26:77:82
            #             Channel:1
            #             Frequency:2.412 GHz (Channel 1)
            #             Quality=43/70  Signal level=-67 dBm  
            #             Encryption key:on
            #             ESSID:"eduroam"
            #             Bit Rates:18 Mb/s; 24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            #             Mode:Master''',
            #     'error':''
            # }
    def getRawNetworkScan(self, sudo=False):
        # Scan command 'iwlist interface scan' needs to be fed as an array.
        if sudo:
            scan_command = ['sudo','iwlist',self.interface,'scan']
        else:
            scan_command = ['iwlist',self.interface,'scan']
        # Open a subprocess running the scan command.
        scan_process = Popen(scan_command, stdout=PIPE, stderr=PIPE)
        # Block all execution, until the scanning completes.
        scan_process.wait()
        # Returns the 'success' and 'error' output.
        (raw_output, raw_error) = scan_process.communicate() 
        # Returns all output in a dictionary for easy retrieval.
        return {'output':raw_output,'error':raw_error}

    # getSSID
        # Description:
            # Parses the 'SSID' for a given cell.
        # -----------------------------------------------
        # Input: (Raw string)
            # 01 - Address: A0:3D:6F:26:77:8E
            # Channel:144
            # Frequency:5.72 GHz
            # Quality=43/70  Signal level=-67 dBm  
            # Encryption key:on
            # ESSID:"ucrwpa"
            # Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            # Mode:Master
        # -----------------------------------------------
        # Returns:
            # 'ucrwpa'
    @staticmethod
    def getSSID(raw_cell):
        ssid = raw_cell.split('ESSID:"')[1]
        ssid = ssid.split('"')[0]
        return ssid

    # getQuality
        # Description:
            # Parses 'Quality level' for a given cell.
        # -----------------------------------------------
        # Input: (Raw string)
            # 01 - Address: A0:3D:6F:26:77:8E
            # Channel:144
            # Frequency:5.72 GHz
            # Quality=43/70  Signal level=-67 dBm  
            # Encryption key:on
            # ESSID:"ucrwpa"
            # Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            # Mode:Master
        # -----------------------------------------------
        # Returns:
            # '43/70'
    @staticmethod
    def getQuality(raw_cell):
        quality = raw_cell.split('Quality=')[1]
        quality = quality.split(' ')[0]
        return quality

    # getSignalLevel
        # Description:
            # Parses 'Signal level' for a given cell.
            # Measurement is in 'dBm'.
        # -----------------------------------------------
        # Input: (Raw string)
            # 01 - Address: A0:3D:6F:26:77:8E
            # Channel:144
            # Frequency:5.72 GHz
            # Quality=43/70  Signal level=-67 dBm  
            # Encryption key:on
            # ESSID:"ucrwpa"
            # Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            # Mode:Master
        # -----------------------------------------------
        # Returns: (string)
            # '-67'    
    @staticmethod
    def getSignalLevel(raw_cell):
        signal = raw_cell.split('Signal level=')[1]
        signal = int(signal.split(' ')[0])
        return signal

    # parseCell
        # Description:
            # Takes a raw cell string and parses it into a dictionary.
        # -----------------------------------------------
        # Input: (Raw string)
            # '''01 - Address: A0:3D:6F:26:77:8E
            # Channel:144
            # Frequency:5.72 GHz
            # Quality=43/70  Signal level=-67 dBm  
            # Encryption key:on
            # ESSID:"ucrwpa"
            # Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            # Mode:Master'''
        # -----------------------------------------------
        # Returns:
            # {
            #     'ssid':'ucrwpa',
            #     'quality':'43/70',
            #     'signal':'-67'
            # }    
    def parseCell(self, raw_cell):
        cell = {
            'ssid': self.getSSID(raw_cell),
            'quality': self.getQuality(raw_cell),
            'signal': self.getSignalLevel(raw_cell)
        }
        return cell

    # formatCells
        # Description:
            # Every network listed is considered a 'cell.
            # This function parses each cell into a dictionary.
            # Returns list of dictionaries. Makes use of 'parseCell'.
            # If not networks were detected, returns False.
        # -----------------------------------------------
        # Input: (Raw terminal string)
            # '''01 - Address: A0:3D:6F:26:77:8E
            # Channel:144
            # Frequency:5.72 GHz
            # Quality=43/70  Signal level=-67 dBm  
            # Encryption key:on
            # ESSID:"ucrwpa"
            # Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            # Mode:Master
            # 02 - Address: A0:3D:6F:26:77:8E
            # Channel:144
            # Frequency:5.72 GHz
            # Quality=30/70  Signal level=-42 dBm  
            # Encryption key:on
            # ESSID:"dd-wrt"
            # Bit Rates:24 Mb/s; 36 Mb/s; 48 Mb/s; 54 Mb/s
            # Mode:Master'''
        # -----------------------------------------------
        # Returns: (Array of dictionaries)
            # [
            #     {
            #         'ssid':'ucrwpa',
            #         'quality':'43/70',
            #         'signal':'-67'
            #     },
            #     {
            #         'ssid':'dd-wrt',
            #         'quality':'30/70',
            #         'signal':'-42'
            #     }
            # ]    
    def formatCells(self, raw_cell_string):
        raw_cells = raw_cell_string.split('Cell') # Divide raw string into raw cells.
        raw_cells.pop(0) # Remove unneccesary "Scan Completed" message.
        if(len(raw_cells) > 0): # Continue execution, if atleast one network is detected.
            # Iterate through raw cells for parsing.
            # Array will hold all parsed cells as dictionaries.
            formatted_cells = [self.parseCell(cell) for cell in raw_cells]
            # Return array of dictionaries, containing cells.
            return formatted_cells
        else:
            print("Networks not detected.")
            return False
        # TODO implement function in ndoe to process this boolean (False)

    # filterAccessPoints
        # Description:
            # If the 'networks' parameter is passed to the 'getAPinfo'
            # function, then this method will filter out all irrelevant 
            # access-points. Access points specified in 'networks' array 
            # will be returned (if available).
        # -----------------------------------------------
        # Input: (Parsed array of cell dictionaries)
            # all_access_points = 
            # [
            #     {
            #         'ssid':'ucrwpa',
            #         'quality':'43/70',
            #         'signal':'-67'
            #     },
            #     {
            #         'ssid':'dd-wrt',
            #         'quality':'30/70',
            #         'signal':'-42'
            #     },
            #     {
            #         'ssid':'linksys',
            #         'quality':'58/70',
            #         'signal':'-24'
            #     }
            # ] 
            # network_names = (array of network names)
            # ['ucrwpa','dd-wrt']
        # -----------------------------------------------
        # Returns: (Array of dictionaries)
            # [
            #     {
            #         'ssid':'ucrwpa',
            #         'quality':'43/70',
            #         'signal':'-67'
            #     },
            #     {
            #         'ssid':'dd-wrt',
            #         'quality':'30/70',
            #         'signal':'-42'
            #     }
            # ] 
    @staticmethod
    def filterAccessPoints(all_access_points, network_names):
        focus_points = [] # Array holding the access-points of concern.
        # Iterate throguh all access-points found.
        for point in all_access_points:
            # Check if current AP is in our desired list.
            if point['ssid'] in network_names:
                focus_points.append(point)
        return focus_points
        # TODO implement something incase our desired ones were not found
 
    # getAPinfo
        # Description:
            # Method returns all (or chosen) available access points (in range).
            # Takes 2 optional parameters: 
            #   'networks' (array): 
            #       Lists all ssid's of concern. Will return only the available access 
            #       points listed here. If not provided, will return ALL access-points in range.        
            #   'sudo' (bool): 
            #       Whether of not method should use sudo privileges. If user uses sudo
            #       privileges, the network manager will be refreshed and will return 
            #       a fresh list of access-points available. If sudo is not provided, 
            #       a cached list will be returned. Cached list gets updated periodically.
        # -----------------------------------------------
        # Input: (Parsed array of cell dictionaries)
            # networks = (array of network names)
            # ['ucrwpa','dd-wrt']
            # sudo = True || False
        # -----------------------------------------------
        # Returns: (Array of dictionaries)
            # [
            #     {
            #         'ssid':'ucrwpa',
            #         'quality':'43/70',
            #         'signal':'-67'
            #     },
            #     {
            #         'ssid':'dd-wrt',
            #         'quality':'30/70',
            #         'signal':'-42'
            #     }
            # ] 
    def getAPinfo(self, networks=False, sudo=False):
        # TODO implement error callback if error is raise in subprocess
        # Unparsed access-point listing. AccessPoints are strings.
        raw_scan_output = self.getRawNetworkScan(sudo)['output'] 
        # Parsed access-point listing. Access-points are dictionaries.
        all_access_points = self.formatCells(raw_scan_output)
        # Checks if access-points were found.
        if all_access_points:
            # Checks if specific networks were declared.
            if networks:
                # Return specific access-points found.
                return self.filterAccessPoints(all_access_points, networks)
            else:
                # Return ALL access-points found.
                return all_access_points
        else:
            # No access-points were found. 
            return False
```

## RSSI Module
### Localization Class

``` python
#!/usr/bin/env python
# RSSI_Localizer
    # Use:
        # from rssi import RSSI_Localizer
        # rssi_localizer_instance = RSSI_Localizer()
    # -------------------------------------------------------
    # Description:
        # This class helps a user implement rssi-based localization.
        # The algorithm assumes the logarithmic distance-path-loss model
        # And assumes a minimum of 3 (or more) access points.
    # -------------------------------------------------------
    # Input:
        # accessPoints: Array holding accessPoint dictionaries.
        #               The order of the arrays supplied will retain
        #               its order, throughout the entire execution.
        # [{
        #     'signalAttenuation': 3, 
        #     'location': {
        #         'y': 1, 
        #         'x': 1
        #     }, 
        #     'reference': {
        #         'distance': 4, 
        #         'signal': -50
        #     }, 
        #     'name': 'dd-wrt'
        # },
        # {
        #     'signalAttenuation': 4, 
        #     'location': {
        #         'y': 1, 
        #         'x': 7
        #     }, 
        #     'reference': {
        #         'distance': 3, 
        #         'signal': -41
        #     }, 
        #     'name': 'ucrwpa'
        # }]
class RSSI_Localizer(object):
    # Allows us to fetch for networks/accessPoints externally.
    # Array of access points must be formatted.
    # 'self.count' parameter is computed internally to aid in 
    # scaling of the algorithm.
    def __init__(self,accessPoints):
        self.accessPoints = accessPoints
        self.count = len(accessPoints)

    # getDistanceFromAP
        # Description:
            # Uses the log model to compute an estimated dstance(di) from node(i)
        # -------------------------------------------------------
        # Input: 
            # accessPoint: dicitonary holding accesspoint info.
            # {
            #     'signalAttenuation': 3, 
            #     'location': {
            #         'y': 1, 
            #         'x': 1
            #     }, 
            #     'reference': {
            #         'distance': 4, 
            #         'signal': -50
            #     }, 
            #     'name': 'dd-wrt'
            # }
            # signalStrength: -69
        # -------------------------------------------------------
        # output: 
            # accessPoint: dicitonary holding accesspoint info.
            # {
            #     'signalAttenuation': 3, 
            #     'location': {
            #         'y': 1, 
            #         'x': 1
            #     }, 
            #     'reference': {
            #         'distance': 4, 
            #         'signal': -50
            #     }, 
            #     'name': 'dd-wrt'
            # }
            # signalStrength: -69,
            # distance: 2
    @staticmethod
    def getDistanceFromAP(accessPoint, signalStrength):
        beta_numerator = float(accessPoint['reference']['signal']-signalStrength)
        beta_denominator = float(10*accessPoint['signalAttenuation'])
        beta = beta_numerator/beta_denominator
        distanceFromAP = round(((10**beta)*accessPoint['reference']['distance']),4)
        accessPoint.update({'distance':distanceFromAP})
        return accessPoint
    
    # TODO fix this because theres two consecutive for loops. 
    # One that runs to fefd signal strengths to this function, 
    # a second consecutive loop inside the function.

    # getDistancesForAllAPs
        # Description:
            # Makes use of 'getDistanceFromAP' to iterate through all 
            # accesspoints being used in localization and obtains the 
            # distance from each one of them.
        # ------------------------------------------------
        # Input:
            # signalStrengths:
            # [siganl1, siganl2, siganl3]
            # [-42, -53, -77]
        # ------------------------------------------------
        # Output:
            # [
            #     {
            #         'distance': 4,
            #         'x': 2,
            #         'y': 3
            #     },
            #     {
            #         'distance': 7,
            #         'x': 2,
            #         'y': 5
            #     },
            #     {
            #         'distance': 9,
            #         'x': 7,
            #         'y': 3
            #     }
            # ]
    def getDistancesForAllAPs(self, signalStrengths):
        apNodes = []
        for i in range(len(self.accessPoints)):
            ap = self.accessPoints[i] 
            distanceFromAP = self.getDistanceFromAP(
                ap,
                signalStrengths[i]
            )
            apNodes.append({
                'distance': distanceFromAP['distance'],
                'x': ap['location']['x'],
                'y': ap['location']['y']
            })
        return apNodes
    
    # createMatrices
        # Description:
            # Creates tehmatrices neccesary to use the least squares method
            # in order to mnimize the error (error=|realDistance-estimatedDistance|). 
            # Assuming 'n' number of nodes and d(m) is the distance(d) from node (m).
            # AX = B, where X is our estimated location.
            # A = [
            #     2(x(i)-xn)    2(y(i)-yn)
            #     2(x(i+1)-xn)  2(y(i+1)-yn)
            #     ...           ...
            #     2(x(n-1)-xn)  2(y(n-1)-yn)
            # ]
            # B = [
            #     x(i)^2 + y(i)^2 - x(n)^2 + y(n)^2 - d(i)^2 + d(n)^2
            #     x(i+1)^2 + y(i+1)^2 - x(n)^2 + y(n)^2 - d(i+1)^2 + d(n)^2
            #     ...
            #     x(n-1)^2 + y(n-1)^2 - x(n)^2 + y(n)^2 - d(n-1)^2 + d(n)^2
            # ]
        # ----------------------------------------
        # Input:
            # accessPoints
            # [
            #     {
            #         'distance': 4,
            #         'x': 2,
            #         'y': 3
            #     },
            #     {
            #         'distance': 7,
            #         'x': 2,
            #         'y': 5
            #     },
            #     {
            #         'distance': 9,
            #         'x': 7,
            #         'y': 3
            #     }
            # ]
        # ----------------------------------------
        # Output:
            # A = [
            #     2(2-7)    2(3-3)
            #     2(2-7)  2(5-3)
            # ]
            # B = [
            #     2^2 + 3^2 - 7^2 + 3^2 - 4^2 + 9^2
            #     2^2 + 5^2 - 7^2 + 3^2 - 7^2 + 9^2
            # ]
    def createMatrices(self, accessPoints):
        # Sets up that te matrics only go as far as 'n-1' rows,
        # with 'n being the # of access points being used.
        n_count = self.count-1
        # initialize 'A' matrix with 'n-1' ranodm rows.
        a = numpy.empty((n_count,2))
        # initialize 'B' matrix with 'n-1' ranodm rows.
        b = numpy.empty((n_count,1))
        # Define 'x(n)' (x of last accesspoint)
        x_n = accessPoints[n_count]['x'] 
        # Define 'y(n)' (y of last accesspoint)
        y_n = accessPoints[n_count]['y']
        # Define 'd(n)' (distance from of last accesspoint)
        d_n = accessPoints[n_count]['distance']
        # Iteration through accesspoints is done upto 'n-1' only
        for i in range(n_count):
            ap = accessPoints[i]
            x, y, d = ap['x'], ap['y'], ap['distance']
            a[i] = [2*(x-x_n), 2*(y-y_n)]
            b[i] = [(x**2)+(y**2)-(x_n**2)-(y_n**2)-(d**2)+(d_n**2)]
        return a, b
    
    # computePosition
        # Description:
            # Performs the 'least squares method' matrix operations 
            # neccessary to get the 'x' and 'y' of the unknown 
            # beacon's position.
            # X = [(A_transposed*A)^-1]*[A_transposed*B]
        # ----------------------------------------
        # Input:
            # A = [
            #     0   0
            #     0  -4
            # ]
            # B = [
            #     4 + 9 - 49 + 9 - 16 + 81  => 38
            #     4 + 25 - 49 + 9 - 49 + 81 => 21
            # ]
        # ----------------------------------------
        # Output:
            # x
            # [
            #     2,
            #     3
            # ]
    @staticmethod
    def computePosition(a, b):
        # Get 'A_transposed' matrix
        at = numpy.transpose(a)
        # Get 'A_transposed*A' matrix
        at_a = numpy.matmul(at,a)
        # Get '[(A_transposed*A)^-1]' matrix
        inv_at_a = numpy.linalg.inv(at_a)
        # Get '[A_transposed*B]'
        at_b = numpy.matmul(at,b)
        # Get '[(A_transposed*A)^-1]*[A_transposed*B]'
        # This holds our position (xn,yn)
        x = numpy.matmul(inv_at_a,at_b) 
        return x

    # getNodePosition
        # Description:
            # Combines 'getDistancesForAllAPs', 'createMatrics',
            # and 'computerPosition' to get the 'X' vector that
            # contains our unkown (x,y) position.
        # ----------------------------------------
        # Input:
            # signalStrengths
            # [4, 2 , 3]
        # ----------------------------------------
        # Output:
            # x
            # [2, 3]
    def getNodePosition(self, signalStrengths):
        apNodes = self.getDistancesForAllAPs(signalStrengths)
        a, b = self.createMatrices(apNodes) 
        position = self.computePosition(a, b)
        # print(a)
        # print(b)
        return position
```

## ROS Wrapper
### RSSI Node

``` python
#!/usr/bin/env python
import rospy #import ROS pip package for using ROS library
from omnibot.msg import MotorArray # import our custom ROS msg types
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

    pub = rospy.Publisher('rssiLocalization_topic', MotorArray, queue_size=10)
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
        # rospy.loginfo(ap_info)
        rssi_values = [ap['signal'] for ap in ap_info]
        rospy.loginfo(rssi_values)
        distances = localizer.getDistancesForAllAPs(rssi_values)
        distances = [distance['distance'] for distance in distances]
        rospy.loginfo(distances)
        position = localizer.getNodePosition(rssi_values)
        # if len(ap_info)<2:
        #     continue
        # rospy.loginfo(rssi_values)
        rospy.loginfo(position)

        # pub.publish(aps[0]['signal'],aps[1]['signal'],0)
            
if __name__ == '__main__':
    try:
        rssiPublisher()
    except rospy.ROSInterruptException:
        pass
```