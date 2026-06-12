// Build-free WebGL point cloud viewer (RViz Orbit-style). Z-colored with the
// same hsl(240->0,70%,55%) ramp as the 2D overlay's CLOUD_PALETTE.
const CR_VS=`attribute vec3 p;uniform mat4 mvp;uniform float zMin,zSpan,size;varying float t;
void main(){gl_Position=mvp*vec4(p,1.0);gl_PointSize=size;t=clamp((p.z-zMin)/zSpan,0.0,1.0);}`;
const CR_FS=`precision mediump float;varying float t;uniform vec4 uColor;
void main(){if(uColor.a>0.0){gl_FragColor=uColor;return;}
vec3 k=clamp(abs(mod((1.0-t)*4.0+vec3(0.0,4.0,2.0),6.0)-3.0)-1.0,0.0,1.0);
gl_FragColor=vec4(0.55+0.63*(k-vec3(0.5)),1.0);}`;
function crPersp(fov,aspect,near,far){const f=1/Math.tan(fov/2),m=new Float32Array(16);m[0]=f/aspect;m[5]=f;m[10]=(far+near)/(near-far);m[11]=-1;m[14]=2*far*near/(near-far);return m}
function crLookAt(e,t,u){let zx=e[0]-t[0],zy=e[1]-t[1],zz=e[2]-t[2],zl=Math.hypot(zx,zy,zz);zx/=zl;zy/=zl;zz/=zl;let xx=u[1]*zz-u[2]*zy,xy=u[2]*zx-u[0]*zz,xz=u[0]*zy-u[1]*zx,xl=Math.hypot(xx,xy,xz);xx/=xl;xy/=xl;xz/=xl;const yx=zy*xz-zz*xy,yy=zz*xx-zx*xz,yz=zx*xy-zy*xx;return new Float32Array([xx,yx,zx,0,xy,yy,zy,0,xz,yz,zz,0,-(xx*e[0]+xy*e[1]+xz*e[2]),-(yx*e[0]+yy*e[1]+yz*e[2]),-(zx*e[0]+zy*e[1]+zz*e[2]),1])}
function crMul(a,b){const o=new Float32Array(16);for(let i=0;i<4;i++)for(let j=0;j<4;j++){let s=0;for(let k=0;k<4;k++)s+=a[k*4+j]*b[i*4+k];o[i*4+j]=s}return o}

class CloudRenderer{
  constructor(canvas){
    this.c=canvas;const gl=this.gl=canvas.getContext('webgl');
    const sh=(type,src)=>{const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);return s};
    const pr=gl.createProgram();gl.attachShader(pr,sh(gl.VERTEX_SHADER,CR_VS));gl.attachShader(pr,sh(gl.FRAGMENT_SHADER,CR_FS));gl.linkProgram(pr);gl.useProgram(pr);
    this.attr=gl.getAttribLocation(pr,'p');gl.enableVertexAttribArray(this.attr);
    this.u={mvp:gl.getUniformLocation(pr,'mvp'),zMin:gl.getUniformLocation(pr,'zMin'),zSpan:gl.getUniformLocation(pr,'zSpan'),size:gl.getUniformLocation(pr,'size'),color:gl.getUniformLocation(pr,'uColor')};
    this.cloudBuf=gl.createBuffer();this.lineBuf=gl.createBuffer();
    this.n=0;this.zMin=0;this.zSpan=1;this.bounds=null;this.robot=null;this.path=[];this.markers=[];
    this.target=[0,0,0];this.dist=20;this.yaw=-Math.PI/2;this.pitch=0.7;this.pointers=new Map();this.pinch=null;
    new ResizeObserver(()=>this.resize()).observe(canvas);this.events();this.resize()
  }
  resize(){const r=this.c.getBoundingClientRect(),d=devicePixelRatio||1;this.c.width=Math.max(1,Math.round(r.width*d));this.c.height=Math.max(1,Math.round(r.height*d));this.draw()}
  setCloud(cloud){
    if(!cloud||!cloud.pts.length)return;
    const pts=cloud.pts,gl=this.gl;
    this.n=pts.length/3|0;this.zMin=cloud.zMin;this.zSpan=Math.max(0.1,cloud.zMax-cloud.zMin);
    let xMin=Infinity,xMax=-Infinity,yMin=Infinity,yMax=-Infinity;
    for(let i=0;i<pts.length;i+=3){if(pts[i]<xMin)xMin=pts[i];if(pts[i]>xMax)xMax=pts[i];if(pts[i+1]<yMin)yMin=pts[i+1];if(pts[i+1]>yMax)yMax=pts[i+1]}
    this.bounds={xMin,xMax,yMin,yMax};
    gl.bindBuffer(gl.ARRAY_BUFFER,this.cloudBuf);gl.bufferData(gl.ARRAY_BUFFER,pts,gl.STATIC_DRAW);
    if(!this.fitted)this.fit();this.draw()
  }
  setRobot(p){this.robot=p;this.draw()} setPath(p){this.path=p;this.draw()} setMarkers(m){this.markers=m;this.draw()}
  fit(){const b=this.bounds;if(!b)return;this.target=[(b.xMin+b.xMax)/2,(b.yMin+b.yMax)/2,this.zMin+this.zSpan/2];this.dist=Math.max(5,Math.hypot(b.xMax-b.xMin,b.yMax-b.yMin)*0.75);this.yaw=-Math.PI/2;this.pitch=0.7;this.fitted=true;this.draw()}
  camera(){const cp=Math.cos(this.pitch),t=this.target;return crLookAt([t[0]+this.dist*cp*Math.cos(this.yaw),t[1]+this.dist*cp*Math.sin(this.yaw),t[2]+this.dist*Math.sin(this.pitch)],t,[0,0,1])}
  draw(){
    const gl=this.gl,w=this.c.width,h=this.c.height,d=devicePixelRatio||1;
    if(!w||!h)return;
    gl.viewport(0,0,w,h);gl.clearColor(0.04,0.055,0.075,1);gl.enable(gl.DEPTH_TEST);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    gl.uniformMatrix4fv(this.u.mvp,false,crMul(crPersp(0.9,w/h,0.1,2000),this.camera()));
    gl.uniform1f(this.u.zMin,this.zMin);gl.uniform1f(this.u.zSpan,this.zSpan);
    if(this.n){
      gl.uniform1f(this.u.size,Math.max(1.5,Math.min(7,60/this.dist))*d);gl.uniform4f(this.u.color,0,0,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,this.cloudBuf);gl.vertexAttribPointer(this.attr,3,gl.FLOAT,false,0,0);gl.drawArrays(gl.POINTS,0,this.n)
    }
    gl.disable(gl.DEPTH_TEST);
    if(this.path.length>1)this.overlay(this.path.flatMap(p=>[p.x,p.y,0.1]),gl.LINE_STRIP,[0.33,0.72,1,1]);
    if(this.markers.length){gl.uniform1f(this.u.size,12*d);this.overlay(this.markers.flatMap(m=>[m.x,m.y,0.2]),gl.POINTS,[0.36,1,0.47,1])}
    if(this.robot){
      const r=this.robot,a=r.yaw||0,z=(r.z||0)+0.15,co=Math.cos(a),si=Math.sin(a);
      this.overlay([r.x+0.45*co,r.y+0.45*si,z,r.x-0.3*co+0.25*si,r.y-0.3*si-0.25*co,z,r.x-0.3*co-0.25*si,r.y-0.3*si+0.25*co,z],gl.TRIANGLES,[1,1,1,1])
    }
  }
  overlay(arr,mode,col){const gl=this.gl;gl.uniform4f(this.u.color,col[0],col[1],col[2],col[3]);gl.bindBuffer(gl.ARRAY_BUFFER,this.lineBuf);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(arr),gl.DYNAMIC_DRAW);gl.vertexAttribPointer(this.attr,3,gl.FLOAT,false,0,0);gl.drawArrays(mode,0,arr.length/3)}
  pan(dx,dy){
    const k=this.dist*0.966/this.c.getBoundingClientRect().height,sy=Math.sin(this.yaw),cy=Math.cos(this.yaw),sp=Math.sin(this.pitch),cp=Math.cos(this.pitch);
    this.target[0]-=-sy*dx*k-(-sp*cy)*dy*k;this.target[1]-=cy*dx*k-(-sp*sy)*dy*k;this.target[2]+=cp*dy*k
  }
  events(){
    this.c.addEventListener('contextmenu',e=>e.preventDefault());
    this.c.addEventListener('wheel',e=>{e.preventDefault();this.dist=Math.max(1,Math.min(800,this.dist*(e.deltaY<0?0.87:1.15)));this.draw()},{passive:false});
    this.c.addEventListener('pointerdown',e=>{this.c.setPointerCapture(e.pointerId);this.pointers.set(e.pointerId,{x:e.clientX,y:e.clientY,pan:e.button!==0||e.shiftKey})});
    this.c.addEventListener('pointermove',e=>{
      const old=this.pointers.get(e.pointerId);if(!old)return;
      const p={x:e.clientX,y:e.clientY,pan:old.pan};this.pointers.set(e.pointerId,p);
      if(this.pointers.size===2){
        const a=[...this.pointers.values()],dd=Math.hypot(a[0].x-a[1].x,a[0].y-a[1].y);
        if(this.pinch){this.dist=Math.max(1,Math.min(800,this.dist*this.pinch.d/dd));this.pan((a[0].x+a[1].x)/2-this.pinch.cx,(a[0].y+a[1].y)/2-this.pinch.cy)}
        this.pinch={d:dd,cx:(a[0].x+a[1].x)/2,cy:(a[0].y+a[1].y)/2}
      }else if(old.pan)this.pan(p.x-old.x,p.y-old.y);
      else{this.yaw-=(p.x-old.x)*0.007;this.pitch=Math.max(-1.5,Math.min(1.5,this.pitch-(p.y-old.y)*0.007))}
      this.draw()
    });
    const up=e=>{this.pointers.delete(e.pointerId);this.pinch=null};
    this.c.addEventListener('pointerup',up);this.c.addEventListener('pointercancel',up)
  }
}
