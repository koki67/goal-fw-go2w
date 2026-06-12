const CLOUD_PALETTE=Array.from({length:256},(_,i)=>{const h=Math.round((1-i/255)*240);return`hsl(${h},70%,55%)`});

class MapRenderer {
  constructor(canvas,onPose){this.c=canvas;this.x=canvas.getContext('2d');this.onPose=onPose;this.grid=null;this.image=null;this.cloudCanvas=null;this.cloudBounds=null;this.robot=null;this.path=[];this.markers=[];this.tool='pan';this.scale=30;this.pan={x:0,y:0};this.drag=null;this.pointers=new Map();new ResizeObserver(()=>this.resize()).observe(canvas);this.events();this.resize()}
  resize(){let r=this.c.getBoundingClientRect(),d=devicePixelRatio||1;this.c.width=r.width*d;this.c.height=r.height*d;this.x.setTransform(d,0,0,d,0,0);this.draw()}
  setGrid(m){this.grid=m.info;let w=m.info.width,h=m.info.height,img=new ImageData(w,h),data=m.data;for(let i=0;i<data.length;i++){let v=data[i],p=((h-1-Math.floor(i/w))*w+(i%w))*4,c=v<0?72:v>=65?25:220;img.data[p]=c;img.data[p+1]=c;img.data[p+2]=v<0?82:c;img.data[p+3]=255}let o=document.createElement('canvas');o.width=w;o.height=h;o.getContext('2d').putImageData(img,0,0);this.image=o;if(!this.fitted)this.fit();this.draw()}
  setCloud(cloud){
    if(!cloud||!cloud.pts.length)return;
    const{pts,zMin,zMax}=cloud,dz=Math.max(0.1,zMax-zMin);
    let xMin=Infinity,xMax=-Infinity,yMin=Infinity,yMax=-Infinity;
    for(let i=0;i<pts.length;i+=3){if(pts[i]<xMin)xMin=pts[i];if(pts[i]>xMax)xMax=pts[i];if(pts[i+1]<yMin)yMin=pts[i+1];if(pts[i+1]>yMax)yMax=pts[i+1]}
    const RES=20,w=Math.ceil((xMax-xMin)*RES)+4,h=Math.ceil((yMax-yMin)*RES)+4;
    const oc=document.createElement('canvas');oc.width=w;oc.height=h;
    const og=oc.getContext('2d');
    for(let i=0;i<pts.length;i+=3){
      const px=(pts[i]-xMin)*RES+2,py=h-(pts[i+1]-yMin)*RES-2;
      const idx=Math.max(0,Math.min(255,Math.round(((pts[i+2]-zMin)/dz)*255)));
      og.fillStyle=CLOUD_PALETTE[idx];og.fillRect(Math.round(px),Math.round(py),2,2)
    }
    this.cloudCanvas=oc;this.cloudBounds={xMin,xMax,yMin,yMax,res:RES,w,h};this.draw()
  }
  setRobot(p){this.robot=p;this.draw()} setPath(p){this.path=p;this.draw()} setMarkers(m){this.markers=m;this.draw()} setTool(t){this.tool=t}
  fit(){if(!this.grid)return;let r=this.c.getBoundingClientRect(),g=this.grid,w=g.width*g.resolution,h=g.height*g.resolution;this.scale=Math.min(r.width/w,r.height/h)*.9;let cx=g.origin.position.x+w/2,cy=g.origin.position.y+h/2;this.pan={x:r.width/2-cx*this.scale,y:r.height/2+cy*this.scale};this.fitted=true;this.draw()}
  screen(p){return{x:this.pan.x+p.x*this.scale,y:this.pan.y-p.y*this.scale}} world(p){return{x:(p.x-this.pan.x)/this.scale,y:-(p.y-this.pan.y)/this.scale}}
  draw(){
    let r=this.c.getBoundingClientRect(),g=this.x;
    g.clearRect(0,0,r.width,r.height);
    if(this.grid&&this.image){let o=this.screen({x:this.grid.origin.position.x,y:this.grid.origin.position.y+this.grid.height*this.grid.resolution});g.imageSmoothingEnabled=false;g.drawImage(this.image,o.x,o.y,this.grid.width*this.grid.resolution*this.scale,this.grid.height*this.grid.resolution*this.scale)}
    if(this.cloudCanvas&&this.cloudBounds){const{xMin,xMax,yMin,yMax,res,w,h}=this.cloudBounds,tl=this.screen({x:xMin-2/res,y:yMax+2/res});g.imageSmoothingEnabled=false;g.drawImage(this.cloudCanvas,tl.x,tl.y,w*this.scale/res,h*this.scale/res)}
    this.line(this.path,'#55b8ff',3);
    for(let m of this.markers){let p=this.screen(m);g.fillStyle=m.color||'#5cff79';g.beginPath();g.arc(p.x,p.y,7,0,Math.PI*2);g.fill()}
    if(this.robot)this.triangle(this.robot,'#fff');
    if(this.drag&&this.tool!=='pan'){this.line([this.drag.start,this.drag.end],'#ffcd57',3);this.triangle({x:this.drag.start.x,y:this.drag.start.y,yaw:Math.atan2(this.drag.end.y-this.drag.start.y,this.drag.end.x-this.drag.start.x)},'#ffcd57')}
  }
  line(ps,color,width){if(ps.length<2)return;let g=this.x,p=this.screen(ps[0]);g.strokeStyle=color;g.lineWidth=width;g.beginPath();g.moveTo(p.x,p.y);for(let q of ps.slice(1)){p=this.screen(q);g.lineTo(p.x,p.y)}g.stroke()}
  triangle(p,color){let q=this.screen(p),a=p.yaw||0,g=this.x;g.save();g.translate(q.x,q.y);g.rotate(-a);g.fillStyle=color;g.beginPath();g.moveTo(14,0);g.lineTo(-9,-8);g.lineTo(-9,8);g.closePath();g.fill();g.restore()}
  events(){this.c.addEventListener('wheel',e=>{e.preventDefault();let b=this.c.getBoundingClientRect(),s={x:e.clientX-b.left,y:e.clientY-b.top},w=this.world(s),f=e.deltaY<0?1.15:.87;this.scale=Math.max(2,Math.min(500,this.scale*f));let n=this.screen(w);this.pan.x+=s.x-n.x;this.pan.y+=s.y-n.y;this.draw()},{passive:false});this.c.addEventListener('pointerdown',e=>{this.c.setPointerCapture(e.pointerId);let p=this.local(e);this.pointers.set(e.pointerId,p);if(this.pointers.size>1)this.multi=true;else this.drag={screen:p,start:this.world(p),end:this.world(p)};this.draw()});this.c.addEventListener('pointermove',e=>{if(!this.drag)return;let p=this.local(e),old=this.pointers.get(e.pointerId);this.pointers.set(e.pointerId,p);if(this.pointers.size===2){let a=[...this.pointers.values()],d=Math.hypot(a[0].x-a[1].x,a[0].y-a[1].y);if(this.pinch)this.scale*=d/this.pinch;this.pinch=d}else if(this.tool==='pan'){this.pan.x+=p.x-old.x;this.pan.y+=p.y-old.y;this.drag.screen=p}else this.drag.end=this.world(p);this.draw()});this.c.addEventListener('pointerup',e=>{this.pointers.delete(e.pointerId);this.pinch=null;if(this.pointers.size===0){if(this.drag&&this.tool!=='pan'&&!this.multi)this.onPose(this.tool,this.drag.start,this.drag.end);this.drag=null;this.multi=false}this.draw()})}
  local(e){let b=this.c.getBoundingClientRect();return{x:e.clientX-b.left,y:e.clientY-b.top}}
}
