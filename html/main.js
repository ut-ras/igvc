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
      url: 'skull.png',
      size: new google.maps.Size(64, 64),
      origin: new google.maps.Point(0,0),
      rotation: 180
    };

    var icon = {
      path: google.maps.SymbolPath.FORWARD_OPEN_ARROW,
      scale: 10,
      rotation: 90
    };
      
    var marker = new google.maps.Marker({
      position: centerLatLng,
      map: map,
      icon: icon
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
      marker.setPosition(new google.maps.LatLng(message.latitude, message.longitude));
    }); 
  
    // Subscribe to yaw
    var ylistener = new ROSLIB.Topic({
      ros : ros,
      name : '/yaw',
      messageType : 'std_msgs/Float64'
    });

    ylistener.subscribe(function(message) {
      var dir = message.data;
      marker.setIcon({
        path: google.maps.SymbolPath.FORWARD_OPEN_ARROW,
        scale: 10,
        rotation: dir*180/Math.PI
      });


    }); 
    
  }

  google.maps.event.addDomListener(window, 'load', initialize);
}());
