#!/usr/bin/env node

module.exports={
//let protocol={
  encode:async function(tf){
    let vecs=await this.tflib.toEuler(tf);
    let euler=vecs[0];
    let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
    let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
    return "["+tarr.concat(rarr).join(",")+"]";
  },
  decode_:function(msg){
    let jss=msg.replace(/\).*/g, ']').replace(/.*\(/, '[').replace(/;/g, '],[').replace(/E\+/g, 'E').replace(/\+/g, '');
    if(jss.length<=2) return [];
    return JSON.parse('['+jss+']');
  },
  decode:async function(msg){
    let ary=this.decode_(msg);
    if(ary.length==0) return [];
    return await this.tflib.fromEuler(ary);
  },
  joints:['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  jdecode:function(msg){
    let jnts=this.decode_(msg);
    return jnts.map((jnt)=>{
      if(jnt.length<this.joints.length) throw new Error("Joint number not enough");
      return jnt.map(function(j){
        return j*Math.PI/180;
      });
    }).slice(0,this.joints.length);
  },
  node:function(n){
    this.tflib.connect(n);
  },
  tflib:null,
  delim:"\n",
  lf:"\n",
  autoclose:false
}

/*
const tflib=require('./tflib');
tflib.set('rzyx deg');
protocol.tflib=tflib;

setImmediate(async function(){
  let tf=await protocol.decode("(1,2,3,30,20,AA)");
  console.log("result "+JSON.stringify(tf));
  let d=await protocol.encode(tf);
  console.log("result "+JSON.stringify(d));
});
*/

