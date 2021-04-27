import googlemaps
import polyline
from geopy import distance

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


gmaps = googlemaps.Client('AIzaSyC7jwTCzP5TEc44lzEfRAtA1FzvKEo2pj8')
distance_threshold = 15

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
                maneuver = 1
            elif step['maneuver'] == 'turn-slight-right':
                maneuver = 2
            elif step['maneuver'] == 'uturn-right':
                maneuver = 3
            elif step['maneuver'] == 'turn-left':
                maneuver = 4
            elif step['maneuver'] == 'turn-slight-left':
                maneuver = 5
            elif step['maneuver'] == 'uturn-left':
                maneuver = 6
            else:
                maneuver = -1
        else:
            maneuver = 0
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
        else:
            return False

request_directions("13 Ahmed Tiesser, Sidi Beshr Bahri, Montaza 2, Alexandria Governorate",
                    "558 Hafez El-Sayed, Fleming, Qism El-Raml, Alexandria Governorate")



