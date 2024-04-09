const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const utils_srvs=ros.require('rovi_utils').srv;

module.exports={
  tf_euler: null,
  node: null,
  option:'rxyz rad',
  connect:async function(n){
    const rosNode=this.node=n;
    if(this.tf_euler==null){
      this.tf_euler=rosNode.serviceClient('/tf_euler/query', utils_srvs.TextFilter, { persist: false });
      if (!await rosNode.waitForService(this.tf_euler.getService(), 2000)) {
        this.tf_euler=null;
        ros.log.error('tf_euler service not available');
        return;
      }
      let req = new utils_srvs.TextFilter.Request();
      req.data=this.option;
      try{
        await this.tf_euler.call(req);
      }
      catch(err){
        ros.log.error('tf_euler call error');
      }
    }
    return this;
  },
  toRT: function(tf) {
    let x=tf.rotation.x;
    let y=tf.rotation.y;
    let z=tf.rotation.z;
    let w=tf.rotation.w;
    let xx=x*x;
    let yy=y*y;
    let zz=z*z;
    let ww=w*w;
    let RT=[[xx-yy-zz+ww,2.*(x*y-w*z),2.*(x*z+w*y),tf.translation.x],
          [2.*(x*y+w*z),yy+ww-xx-zz,2.*(y*z-w*x),tf.translation.y],
          [2.*(x*z-w*y),2.*(y*z+w*x),zz+ww-xx-yy,tf.translation.z],
          [0,0,0,1]];
    return RT;
  },
  fromRT: function(rt) {
    let s=qw=qx=qy=qz=0.0;
    if (rt[0][0]+rt[1][1]+rt[2][2]>0){
      s=Math.sqrt(1.0+rt[0][0]+rt[1][1]+rt[2][2])*2;  //s=qw*4
      qw=s/4;
      qx=(rt[2][1]-rt[1][2])/s;
      qy=(rt[0][2]-rt[2][0])/s;
      qz=(rt[1][0]-rt[0][1])/s;
    } else if ((rt[0][0]>rt[1][1]) && (rt[0][0]>rt[2][2])) {
      s=Math.sqrt(1.0+rt[0][0]-rt[1][1]-rt[2][2])*2;  //s=qx*4
      qw=(rt[2][1]-rt[1][2])/s;
      qx=s/4;
      qy=(rt[0][1]+rt[1][0])/s;
      qz=(rt[0][2]+rt[2][0])/s;
    } else if (rt[1][1]>rt[2][2]) {
      s=Math.sqrt(1.0-rt[0][0]+rt[1][1]-rt[2][2])*2;  //s=qy*4
      qw=(rt[0][2]-rt[2][0])/s;
      qx=(rt[0][1]+rt[1][0])/s;
      qy=s/4;
      qz=(rt[1][2]+rt[2][1])/s;
    } else {
      s=Math.sqrt(1.0-rt[0][0]-rt[1][1]+rt[2][2])*2;  //s=qz*4
      qw=(rt[1][0]-rt[0][1])/s;
      qx=(rt[0][2]+rt[2][0])/s;
      qy=(rt[1][2]+rt[2][1])/s;
      qz=s/4;
    }
    let tf=new geometry_msgs.Transform();
    tf.rotation.w=qw;
    tf.rotation.x=qx;
    tf.rotation.y=qy;
    tf.rotation.z=qz;
    tf.translation.x=rt[0][3];
    tf.translation.y=rt[1][3];
    tf.translation.z=rt[2][3];
    return tf;
  },
  toEuler: async function(tfs){
    let req=new utils_srvs.TextFilter.Request();
    tfs.forEach(function(tf){
      req.data+=tf.rotation.x+','+tf.rotation.y+','+tf.rotation.z+','+tf.rotation.w+'\n';
    });
    req.data=req.data.trim();
    let res;
    try{
      res=await this.tf_euler.call(req);
    }
    catch(err) {
      ros.log.error('tf_euler call error');
      return [];
    }
    let vecs=res.data.split('\n').map(function(v,i){
      let tf=tfs[i];
      let vec=[tf.translation.x,tf.translation.y,tf.translation.z];
      return vec.concat(JSON.parse('['+v+']'));
    });
    return vecs;
  },
  fromEuler: async function(pos){
    let req = new utils_srvs.TextFilter.Request();
    req.data=pos.map(function(p){ return p.slice(3).toString();}).join('\n');
    let res;
    try{
      res=await this.tf_euler.call(req);
      let tfs=res.data.split('\n').map(function(v,i){
        let tf=new geometry_msgs.Transform();
        tf.translation.x=pos[i][0];
        tf.translation.y=pos[i][1];
        tf.translation.z=pos[i][2];
        let elm=v.split(',');
        tf.rotation.x=Number(elm[0]);
        tf.rotation.y=Number(elm[1]);
        tf.rotation.z=Number(elm[2]);
        tf.rotation.w=Number(elm[3]);
        return tf;
      });
      return tfs;
    }
    catch(err) {
      ros.log.error('tf_euler call error');
      return [];
    }
  }
};

