//***************Below is the calculator for distance && bearing***************
      float R = 6371000; //radius of earth in meters
      float lat1 = 30.4018; //put your target's latitude here
      float lon1 = -97.8913; //put your target's longitude here
      float lat2 = GPS.latitudeDegrees; //expecting current location from GPS module
      float lon2 = GPS.longitudeDegrees; 

      //calculate bearing and remove negative degrees
      float brng = degrees(atan2((sin(radians(lon2-lon1)) * cos(radians(lat2))), (cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(lon2-lon1)))));
      while (brng < 0) 
        brng += 360;

      //calculate distance to target in meters
      float a = sin(radians(lat2-lat1)/2) * sin(radians(lat2-lat1)/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(radians(lon2-lon1)/2) * sin(radians(lon2-lon1)/2);
      float d = R * 2 * atan2(sqrt(a), sqrt(1-a));

//Finishes with 'd' for the distance variable and 'brng' for the bearing variable 
