const $=id=>document.getElementById(id);let ros,topics=[],mode='detecting',subscriptions=[],modeTimer=null,locState='',goalState='';
const renderer=new MapRenderer($('map'),publishPose);
function badge(text,good=false){$('connection').textContent=text;$('connection').className='badge '+(good?'good':'bad')}
function connect(){let port=new URLSearchParams(location.search).get('rosbridge_port')||'9090';ros=new ROSLIB.Ros({url:`ws://${location.hostname}:${port}`});ros.on('connection',()=>{badge('CONNECTED',true);setup()});ros.on('close',()=>{badge('DISCONNECTED');clearSubs();setTimeout(connect,2000)});ros.on('error',()=>badge('CONNECTION ERROR'))}
function clearSubs(){subscriptions.forEach(t=>t.unsubscribe());subscriptions=[];clearInterval(modeTimer);modeTimer=null}
function topic(name,type,cb,opt={}){let t=new ROSLIB.Topic({ros,name,messageType:type,compression:opt.compression||'none',throttle_rate:opt.throttle||0});t.subscribe(cb);subscriptions.push(t);return t}
function latched(name,type,cb,opt={}){let got=false,t=topic(name,type,m=>{got=true;cb(m)},opt),retry=setInterval(()=>{if(got||!ros||!ros.isConnected){clearInterval(retry);return}t.unsubscribe();t.subscribe(m=>{got=true;cb(m)})},3000);return t}
function setup(){clearSubs();mode='detecting';ros.getTopics(result=>{topics=result.topics||[];detect();subscribeCommon();clearInterval(modeTimer);modeTimer=setInterval(pollMode,5000)})}
function pollMode(){if(!ros||!ros.isConnected)return;ros.getTopics(r=>{topics=r.topics||[];detect()})}
function detect(){let next=topics.includes('/map')?'navigation':topics.includes('/web/prep_grid')?'preparation':'detecting';if(next===mode)return;mode=next;$('mode').textContent=mode.toUpperCase();$('nav-controls').classList.toggle('hidden',mode!=='navigation');$('prep-controls').classList.toggle('hidden',mode!=='preparation');subscribeMode();updateHint()}
function subscribeCommon(){latched('/localization/state','std_msgs/String',m=>{$('localization').textContent=m.data;locState=m.data;updateHint()});latched('/goal_executor/status','std_msgs/String',m=>{$('goal-status').textContent=m.data;goalState=m.data;updateHint()});topic('/localization/fitness','std_msgs/Float32',m=>$('fitness').textContent=Number(m.data).toFixed(3),{throttle:1000})}
function subscribeMode(){
  if(mode==='navigation'){
    latched('/map','nav_msgs/OccupancyGrid',m=>renderer.setGrid(m),{compression:'cbor'});
    latched('/map_cloud','sensor_msgs/PointCloud2',m=>{const c=decodePC2(m);if(c)renderer.setCloud(c)},{compression:'cbor'});
    topic('/localization/pose','geometry_msgs/PoseWithCovarianceStamped',m=>renderer.setRobot(pose(m.pose.pose)),{throttle:200});
    topic('/plan','nav_msgs/Path',m=>renderer.setPath(m.poses.map(p=>({x:p.pose.position.x,y:p.pose.position.y}))),{throttle:500});
    topic('/goal_markers','visualization_msgs/MarkerArray',m=>renderer.setMarkers(m.markers.filter(x=>x.type===0||x.type===2).map(x=>({x:x.pose.position.x,y:x.pose.position.y,color:rgba(x.color)}))))
  }else if(mode==='preparation'){
    latched('/web/prep_grid','nav_msgs/OccupancyGrid',m=>renderer.setGrid(m),{compression:'cbor'});
    topic('/dlio/odom_node/odom','nav_msgs/Odometry',m=>renderer.setRobot(pose(m.pose.pose)),{throttle:200});
    latched('/web/prep_status','std_msgs/String',m=>{$('prep-status').textContent=m.data;$('finish').disabled=/^(SAVING|CONVERTING)/.test(m.data)})
  }
}
function yaw(q){return Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}function pose(p){return{x:p.position.x,y:p.position.y,yaw:yaw(p.orientation)}}function rgba(c){return`rgba(${c.r*255},${c.g*255},${c.b*255},${c.a})`}
function decodePC2(msg){
  const f={};for(const x of msg.fields)f[x.name]=x;
  if(!f.x||!f.y||!f.z)return null;
  const raw=msg.data,step=msg.point_step,n=msg.width*msg.height,little=!msg.is_bigendian;
  let buf;if(raw.buffer)buf=raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength);else buf=new Uint8Array(raw).buffer;
  const view=new DataView(buf),pts=new Float32Array(n*3);
  let count=0,zMin=Infinity,zMax=-Infinity;
  for(let i=0;i<n;i++){const b=i*step,x=view.getFloat32(b+f.x.offset,little),y=view.getFloat32(b+f.y.offset,little),z=view.getFloat32(b+f.z.offset,little);if(!isFinite(x)||!isFinite(y)||!isFinite(z))continue;pts[count++]=x;pts[count++]=y;pts[count++]=z;if(z<zMin)zMin=z;if(z>zMax)zMax=z}
  return{pts:pts.subarray(0,count),zMin,zMax}
}
function updateHint(){
  const tool=renderer.tool;
  if(mode!=='navigation'){$('hint').textContent=mode==='preparation'?'Drive to build map coverage. Press "Finish & Save" when done.':'Waiting for navigation map...';return}
  if(tool==='initial'){$('hint').textContent='Drag from the robot\'s position toward its heading, then release.';return}
  if(tool==='goal'){$('hint').textContent='Drag from the destination toward the arrival heading, then release.';return}
  const active=locState==='TRACKING'||locState==='DEGRADED';
  if(!active){
    $('hint').textContent=locState==='CONVERGING'||locState==='LOST'
      ?`Localization: ${locState} — click "2D Pose Estimate" again if it does not converge within ~5 s.`
      :'Step 1 — Click "2D Pose Estimate", then drag from the robot\'s position toward its heading.';
  }else if(goalState==='ACTIVE'||goalState==='PREEMPTING'){
    $('hint').textContent='Navigation active — click "2D Nav Goal" to send a new destination (preempts current).';
  }else{
    $('hint').textContent='Step 2 — Click "2D Nav Goal", then drag from the destination toward the arrival heading.';
  }
}
function publishPose(tool,start,end){if(mode!=='navigation')return;let a=Math.atan2(end.y-start.y,end.x-start.x),q={x:0,y:0,z:Math.sin(a/2),w:Math.cos(a/2)},stamp={secs:0,nsecs:0};if(tool==='goal')new ROSLIB.Topic({ros,name:'/goal_pose',messageType:'geometry_msgs/PoseStamped'}).publish({header:{stamp,frame_id:'map'},pose:{position:{x:start.x,y:start.y,z:0},orientation:q}});if(tool==='initial'){let cov=Array(36).fill(0);cov[0]=.25;cov[7]=.25;cov[35]=.0685;new ROSLIB.Topic({ros,name:'/initialpose',messageType:'geometry_msgs/PoseWithCovarianceStamped'}).publish({header:{stamp,frame_id:'map'},pose:{pose:{position:{x:start.x,y:start.y,z:0},orientation:q},covariance:cov}})}renderer.setTool('pan');$('tool').textContent='Tool: pan';updateHint()}
$('initial').onclick=()=>{renderer.setTool('initial');$('tool').textContent='Tool: 2D Pose Estimate';updateHint()};
$('goal').onclick=()=>{renderer.setTool('goal');$('tool').textContent='Tool: 2D Nav Goal';updateHint()};
$('fit').onclick=()=>renderer.fit();$('finish').onclick=()=>{if(!confirm('Save and convert the current map?'))return;$('finish').disabled=true;new ROSLIB.Service({ros,name:'/web/finish_map',serviceType:'std_srvs/Trigger'}).callService({},r=>{if(!r.success)alert(r.message);$('finish').disabled=false})};connect();
