(function () {
  function initialize() {
    var centerLatLng = new google.maps.LatLng(30.288157, -97.735669);

    var mapOptions = {
      center: centerLatLng,
      zoom: 20
    };
  
    var map = new google.maps.Map(document.getElementById("map-canvas"),
        mapOptions);
 
    var image = {
      url: 'circle.png',
      size: new google.maps.Size(20, 20),
      origin: new google.maps.Point(0,0),
    };
      
    var marker = new google.maps.Marker({
      position: centerLatLng,
      map: map,
      icon: image
    });
  
    // Connect to ROS
    var ros = new ROSLIB.Ros({
      url : 'ws://10.145.36.234:9090'
    });

    // Subscribe to gps
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/gps/trimble/raw',
      messageType : 'gps_common/GPSFix'
    });

    listener.subscribe(function(message) {
      console.log('Received message on ' + listener.name + ': ' + message);
      marker.setPosition(new google.maps.LatLng(message.latitude, message.longitude));
    }); 
  
    // Subscribe to yaw
    var ylistener = new ROSLIB.Topic({
      ros : ros,
      name : '/orientation_data',
      messageType : 'std_msgs/Float64'
    });

    ylistener.subscribe(function(message) {
      console.log('Received message on ' + listener.name + ': ' + message);
    }); 
    
  }

  google.maps.event.addDomListener(window, 'load', initialize);
}());
