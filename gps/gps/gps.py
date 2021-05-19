import rclpy
from rclpy.node import Node

from message_object.msg import MessageObject                           


import googlemaps
import polyline
from geopy import distance

gmaps = googlemaps.Client('AIzaSyC7jwTCzP5TEc44lzEfRAtA1FzvKEo2pj8')
distance_threshold = 15
my_location = (31.237545,29.961635)
to_location = (31.270491,30.000263)

class step_info:
    def __init__(self, polyline, end_location, maneuver, duration):
        self.polyline = polyline # array of tupple (lat, lng)
        self.end_location = end_location #tupple (lat ,lng)
        self.maneuver = maneuver # int //enum
        self.duration = duration # int (sec)
        # print(type(polyline[0][0]))
        # print(type(end_location[0]))
        # print(type(maneuver))
        # print(type(duration))

def request_directions(source, destination):
    request_result = []
    routes = gmaps.directions(source, destination, mode="driving")
    steps = routes[0]['legs'][0]['steps']

    for step in steps:
        decoded_polyline = polyline.decode(step['polyline']['points'])
        lat = step['end_location']['lat']
        lng = step['end_location']['lng']
        end_location = (lat, lng)
        if "maneuver" in step:
            if step['maneuver'] == 'turn-right':
                maneuver = MessageObject.TURN_RIGHT
            elif step['maneuver'] == 'turn-slight-right':
                maneuver = MessageObject.TURN_SLIGHT_RIGHT
            elif step['maneuver'] == 'uturn-right':
                maneuver = MessageObject.UTURN_RIGHT
            elif step['maneuver'] == 'turn-left':
                maneuver = MessageObject.TURN_LEFT
            elif step['maneuver'] == 'turn-slight-left':
                maneuver = MessageObject.TURN_SLIGHT_RIGHT
            elif step['maneuver'] == 'uturn-left':
                maneuver = MessageObject.UTURN_LEFT
            else:
                maneuver = MessageObject.ELSE
        else:
            maneuver = MessageObject.HEAD
        duration = step['duration']['value']
        new_step = step_info(decoded_polyline, end_location, maneuver, duration)
        request_result.append(new_step)
    
    return request_result



def get_distance(point1, point2):
    D = distance.distance(point1, point2).m
    return D


def check_if_on_path(current_location, polyline):
    for point in polyline:
        D = get_distance(current_location, point)
        if D < distance_threshold:
            return True
        # else:
        #     return False
    return False

# request_directions("13 Ahmed Tiesser, Sidi Beshr Bahri, Montaza 2, Alexandria Governorate",
#                     "558 Hafez El-Sayed, Fleming, Qism El-Raml, Alexandria Governorate")

def set_message(message , step : step_info):
    message.duration = step.duration
    message.end_location_lat = step.end_location[0]
    message.end_location_lng = step.end_location[1]
    message.distance_remaining_in_step = get_distance(step.end_location , my_location)
    message.polyline_lng = []
    message.polyline_lat = []

    for i in range(len(step.polyline)):
        message.polyline_lat.append(step.polyline[i][0])
        message.polyline_lng.append(step.polyline[i][1])

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.message_array = request_directions(my_location , to_location)
        self.get_logger().info(' google maps request made ')
        self.i = 0
        self.n = len(self.message_array)
        self.publisher_ = self.create_publisher(MessageObject, 'GPStopic', 10)  
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        if get_distance(self.message_array[-1].end_location , my_location) < distance_threshold:
            self.get_logger().info('reached destination' )
            self.destroy_node()
            return

        msg = MessageObject()
        msg.next_maneuver = MessageObject.HEAD
        if self.i < self.n:                
            if check_if_on_path(my_location , self.message_array[self.i].polyline ):
                set_message(msg ,self.message_array[self.i])
                if self.i + 1 < self.n: 
                    msg.next_maneuver = self.message_array[self.i+1].maneuver
                
               
            elif self.i + 1 < self.n:
                if check_if_on_path(my_location , self.message_array[self.i+1].polyline ):
                    self.i += 1
                    set_message(msg ,self.message_array[self.i])
                    if self.i + 1 < self.n: 
                        msg.next_maneuver = self.message_array[self.i+1].maneuver
                else :
                    self.get_logger().info('strayed out of path recalculating')
                    self.message_array = request_directions(my_location , to_location)
                    self.get_logger().info(' google maps request made ')
                    self.i = 0
                    self.n = len(self.message_array)

            else :
                self.get_logger().info('strayed out of path recalculating')
                self.message_array = request_directions(my_location , to_location)
                self.get_logger().info(' google maps request made ')
                self.i = 0
                self.n = len(self.message_array)
            
            self.publisher_.publish(msg)
            self.get_logger().info('talker : maneuver: "%d" ' % msg.next_maneuver )    
            self.get_logger().info('talker : destination lng: "%.6f" ' % msg.end_location_lat  )
            self.get_logger().info('talker : destination lat: "%.6f" ' % msg.end_location_lng )
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
