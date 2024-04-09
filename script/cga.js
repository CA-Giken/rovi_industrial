#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='sxyz deg';

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  if(ary.length==0) return [];
  let aryJ5=JSON.parse(JSON.stringify(ary));
  ary=ary.map(function(a){
    a[3]*=-1;
    a[4]=-a[5];
    a[5]=0.;
    return a;
  });
  aryJ5=aryJ5.map(function(a){
    a[0]=0.;
    a[1]=0.;
    a[2]=0.;
    a[3]=0.;
    a[4]=0.;
    return a;
  });
  console.log(ary);
  console.log(aryJ5);
  return await this.tflib.fromEuler(ary);
}
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
  let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
  return tarr.concat(rarr).join(",");
}

protocol.delim=",";
protocol.lf="";

module.exports=protocol;
