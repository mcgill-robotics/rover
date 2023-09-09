import math

def haversine_distance(lat1, long1, lat2, long2):
    R = 6371.0  # Earth's radius in kilometers
    
    lat1_rad = math.radians(lat1)
    long1_rad = math.radians(long1)
    lat2_rad = math.radians(lat2)
    long2_rad = math.radians(long2)
    
    delta_lat = lat2_rad - lat1_rad
    delta_long = long2_rad - long1_rad
    
    a = math.sin(delta_lat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_long / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    
    return distance

# Coordinates
lat1 = 51.42275619506836
long1 = -112.64131164550781
lat2 = 51.42307662963867
long2 = -112.64105224609375

distance = haversine_distance(lat1, long1, lat2, long2)
print(distance*1000)
