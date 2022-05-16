const ros = new ROSLIB.Ros();


var topics = []
var types = []


const getTopicList = () => {
    var client = new ROSLIB.Service({
      ros: ros,
      name: "rosapi/topics",
      serviceType: "rosapi/Topics"
    })
  
    var req = new ROSLIB.ServiceRequest();
  
    client.callService(req, function(res){

        sel = document.getElementById("select")

        topics = res.topics
        types = res.types

        for (let index = 0; index < topics.length; index++) {
            var opt = document.createElement('option');
            opt.value = index;
            opt.innerHTML = topics[index];
            sel.appendChild(opt);     
        }

    })
}
  

const changeEvent = () => {
    value = document.getElementById("select").value;

    document.getElementById("name").value = topics[value]
    document.getElementById("type").value = types[value]
}



ros.connect('ws://localhost:9090');


ros.on('error', (error) => {
  console.log(error);
});


ros.on('connection', (error) => {
  console.log('Connection: ok!');
});


ros.on('close', (error) => {
  console.log('Connection closed.');
});


window.onload = () => {
    getTopicList()
}

