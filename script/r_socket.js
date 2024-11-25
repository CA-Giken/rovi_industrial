#!/usr/bin/env node

let namespace='rsocket';
const net=require('net')
const ping=require('ping')
const EventEmitter=require('events').EventEmitter;
const spawn=require('child_process').spawn;
const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;
const utils_srvs=ros.require('rovi_utils').srv;
let tf_lookup=null;
let Xable=true;

let Config={
  protocol:'fanuc',
  port:8888,
  socket_timeout:0,
  delay:100,
  capt_timeout:10,
  solve_timeout:10,
  recipe_timeout:3,
  base_frame_id:'base',
  source_frame_id:'camera/master0',
  target_frame_id:'solve0',
  update_frame_id:'',
  check_frame_id:'',
  rebase_frame_id:'baseVT',
  fitness:'fitness',
  x1keys:[],
  x1interlock:'',
  reverse_frame_id:'',
  reverse_direction:1,
};
let Param={
  post:'',
  x1interlock:true
};
let Score={'fitness':[0,0]};

setImmediate(async function(){
  console.log("r-socket start:"+namespace);
  const emitter=new EventEmitter();
  const rosNode=await ros.initNode(namespace);
  console.log("r-socket registered:");
  try{
    let co=await rosNode.getParam('/config/'+namespace);
    Object.assign(Config,co);
  }
  catch(e){
  }
  const protocol=require('./'+Config.protocol+'.js');
  protocol.node(rosNode);
//ROS events//////////////////
  rosNode.subscribe('/response/clear',std_msgs.Bool,async function(ret){
    emitter.emit('clear',ret.data);
  });
  rosNode.subscribe('/response/capture',std_msgs.Bool,async function(ret){
    emitter.emit('capture',ret.data);
  });
  rosNode.subscribe('/response/solve',std_msgs.Bool,async function(ret){
    emitter.emit('solve',ret.data);
  });
  rosNode.subscribe('/response/recipe_load',std_msgs.Bool,async function(ret){
    emitter.emit('recipe',ret.data);
  });
  rosNode.subscribe('/rsocket/enable',std_msgs.Bool,async function(ret){
    Xable=ret.data;
  });
  rosNode.subscribe('/report',std_msgs.String,function(msg){
    let jss=msg.data.replace(/\)/g, ']').replace(/\(/g, '[').replace(/\'/g, '\"');
    Object.assign(Score,JSON.parse(jss));
  });
  const pub_conn=rosNode.advertise('/rsocket/stat',std_msgs.Bool);
  const pub_tf=rosNode.advertise('/update/config_tf',geometry_msgs.TransformStamped);
  const pub_clear=rosNode.advertise('/request/clear',std_msgs.Bool);
  const pub_capture=rosNode.advertise('/request/capture',std_msgs.Bool);
  const pub_solve=rosNode.advertise('/request/solve',std_msgs.Bool);
  const pub_recipe=rosNode.advertise('/request/recipe_load',std_msgs.String);
  const pub_report=rosNode.advertise('/report',std_msgs.String);
  rosNode.subscribe(namespace+'/ping',std_msgs.Bool,async function(ret){
    let res = await ping.promise.probe(Config.robot_ip,{timeout:3});
    if(res.alive){
      let stat=new std_msgs.Bool();
      stat.data=true;
      pub_conn.publish(stat);
    }
  });
  tf_lookup=rosNode.serviceClient('/tf_lookup/query', utils_srvs.TextFilter, { persist: false });
  if (!await rosNode.waitForService(tf_lookup.getService(), 2000)) {
    ros.log.error('tf_lookup service not available');
    return;
  }
//Function///////////////
  let reverse_frame_updater=null;
//Rotate J7 by J6////////////////
  if(Config.reverse_frame_id.length>0){
    let rf=Config.reverse_frame_id;
    let rd=Config.reverse_direction;
    let ctf=await rosNode.getParam('/config_tf');
    if(ctf.hasOwnProperty(rf)){
      reverse_frame_updater=function(j6){
        let tf=new geometry_msgs.TransformStamped();
        tf.header.stamp=ros.Time.now();
        tf.header.frame_id=ctf[rf].parent_frame_id;
        tf.child_frame_id=rf;
        let rot2=j6*0.5*rd;
        tf.transform.rotation.x=0;
        tf.transform.rotation.y=0;
        tf.transform.rotation.z=Math.sin(rot2);
        tf.transform.rotation.w=Math.cos(rot2);
        pub_tf.publish(tf);
      }
      rosNode.subscribe('/joint_states',sensor_msgs.JointState,async function(joint){
        reverse_frame_updater(joint.position[joint.position.length-1]);
      });
    }
  }

  let stat_out_timer=null;
  function stat_out(f){
    let stat=new std_msgs.Bool();
    stat.data=f;
    pub_conn.publish(stat);
    if(f){
      if(stat_out_timer==null) stat_out_timer=setInterval(function(){ stat_out(f);},500);
    }
    else{
      if(stat_out_timer!=null){
        clearInterval(stat_out_timer);
        stat_out_timer=null;
      }
    }
  }

//Socket events//////////////////
  function respNG(conn,proto,err){
    let l1='NG'+proto.delim;
    let l2=''+err+proto.lf;
    if(proto.delim==proto.lf){ conn.write(l1); conn.write(l2);}
    else conn.write(l1+l2);
    if(proto.autoclose) conn.destroy();
    conn.x012=false;
    stat_out(false);
    let rep=new std_msgs.String();
    log_date=""
    log_time=""
    log_date,log_time = get_time();
    rep.data=JSON.stringify({error:[err,1]});
    pub_report.publish(rep);
  }
  function respOK(conn,proto,cod){
    switch(arguments.length){
    case 2:
      conn.write('OK'+proto.lf);
      break;
    case 3:
      let l1='OK'+proto.delim;
      let l2=cod+proto.lf;
      if(proto.delim==proto.lf){ conn.write(l1); conn.write(l2);}
      else conn.write(l1+l2);
      break;
    }
    if(proto.autoclose) conn.destroy();
    conn.x012=false;
    stat_out(false);
    let rep=new std_msgs.String();
    log_date=""
    log_time=""
    log_date,log_time = get_time();
    rep.data=JSON.stringify({error:[0,0]});
    pub_report.publish(rep);
  }
  function respOK_x2(conn,proto,cod){
    switch(arguments.length){
    case 2:
      conn.write('OK'+proto.lf);
    break;
    case 3:
      let l1='OK'+proto.delim;
      let l2=cod+proto.lf;
      if(proto.delim==proto.lf){ conn.write(l1); conn.write(l2);}
      else conn.write(l1+l2);
    break;
    }
    if(proto.autoclose) conn.destroy();
    conn.x012=false;
    stat_out(false);
    let rep=new std_msgs.String();
    log_date=""
    log_time=""
    log_date,log_time = get_time();
    const cod_list = cod.split(',');
    rep.data=JSON.stringify({error:[0,0],day:log_date,time:log_time,
      hosei_x:Number(cod_list[0]),hosei_y:Number(cod_list[1]),hosei_z:Number(cod_list[2]),
      hosei_rx:Number(cod_list[3]),hosei_ry:Number(cod_list[4]),hosei_rz:Number(cod_list[5])});
    pub_report.publish(rep);
  }

  function get_time(date){
    let now = new Date();
    var Year = now.getFullYear();
    var Month = now.getMonth() + 1;
    var Day = now.getDate();
    var Hour = now.getHours();
    var Min = now.getMinutes();
    var Sec = now.getSeconds();
    log_date = Year + "年" + Month + "月" + Day + "日";
    log_time = Hour + "時" + Min + "分" + Sec + "秒";
    ros.log.info("日付 : " + log_date);
    ros.log.info("時刻 : " + log_time);    
    return log_date,log_time;
  }

  function tf_update(tf,id){
    let stmp=new geometry_msgs.TransformStamped();
    stmp.header.stamp=ros.Time.now();
    stmp.header.frame_id='';
    stmp.child_frame_id=id;
    stmp.transform=tf;
    return stmp;
  }
  let TX1;
  const server = net.createServer(function(conn){
    conn.setTimeout(Config.socket_timeout*1000);
    let buffer='';
    let msg='';
    let wdt=null;
    let stat;
    conn.x012=false;
    function X0(){
      let f=new std_msgs.Bool();
      f.data=true;
      pub_clear.publish(f);
      setTimeout(function(){
        if(conn.x012) setImmediate(X1);
        else respOK(conn,protocol);
      },100);
    }
    async function X1(){
      let t0=ros.Time.now();
      TX1=t0;
      if(!Param.x1interlock){
        respNG(conn,protocol,913);
        return;
      }
      if(Config.x1keys.length>0){  //init as magic number
        Score[Config.x1keys[0]]=99999;
      }
      let tfs;
      try{
        tfs=await protocol.decode(msg.substr(2).trim());
      }
      catch(e){
        ros.log.error('r_socket::pose decode error '+e);
        respNG(conn,protocol,999); //decode error
        return;
      }
      if(tfs.length>0 && tfs[0].hasOwnProperty('translation') && Config.update_frame_id.length>0){
        ros.log.warn("rsocket::X1::update_frame"+JSON.stringify(tfs));
        let tf=tf_update(tfs[0],Config.update_frame_id);
        pub_tf.publish(tf);
      }
      setTimeout(function(){
        let f=new std_msgs.Bool();
        f.data=true;
        pub_capture.publish(f);
      },Config.delay);
      wdt=setTimeout(function(){
        wdt=null;
        emitter.removeAllListeners('capture');
        ros.log.warn("rsocket::capture timeout");
        respNG(conn,protocol,911); //capture timeout
      },Config.capt_timeout*1000);
      emitter.removeAllListeners('capture');
      let x1retries=0;
      let x1callback;
      emitter.once('capture',x1callback=function(ret){
        clearTimeout(wdt);
        wdt=null;
        let t1=ros.Time.now();
        ros.log.info("rsocket::capture done "+ret+" "+(ros.Time.toSeconds(t1)-ros.Time.toSeconds(t0)));
        if(ret){
          if(conn.x012) setImmediate(X2);
          else{
            if(Config.x1keys.length==0) respOK(conn,protocol);
            else{
              if(Score[Config.x1keys[0]]==99999){  //Score never subscribed
                x1retries++;
                ros.log.info("rsocket::x1callback retry "+x1retries);
                if(x1retries<3){
                  setTimeout(function(){
                    x1callback(ret);
                  },300);
                  return;
                }
                else{
                  respNG(conn,protocol,914); //failed
                }
              }
              let vals=Config.x1keys.map((k)=>{
                return Score.hasOwnProperty(k)? Score[k]:0;
                });
              respOK(conn,protocol,'['+vals.toString()+']');
              }
           }
        }
        else respNG(conn,protocol,912); //failed to capture
      });
    }
    async function X2(){
      let t0=ros.Time.now();
      let tfs;
      try{
        tfs=await protocol.decode(msg.substr(2).trim());
      }
      catch(e){
        ros.log.error('r_socket::pose decode error '+e);
        respNG(conn,protocol,999); //decode error
        return;
      }
      ros.log.warn("rsocket::X2 tf parse "+JSON.stringify(tfs));
      if(tfs.length>0 && tfs[0].hasOwnProperty('translation') && Config.check_frame_id.length>0){
        let tf=tf_update(tfs[0],Config.check_frame_id);
        pub_tf.publish(tf);
      }

      let f=new std_msgs.Bool();
      f.data=true;
      pub_solve.publish(f);
      wdt=setTimeout(function(){
        wdt=null;
        emitter.removeAllListeners('solve');
        ros.log.warn("rsocket::solve timeout");
        respNG(conn,protocol,921); //solve timeout
      },Config.solve_timeout*1000);
      emitter.removeAllListeners('solve');
      emitter.once('solve',async function(ret){
        clearTimeout(wdt);
        wdt=null;
        try{
          let t1=ros.Time.now();
          let tx2=ros.Time.toSeconds(t1)-ros.Time.toSeconds(t0);
          let tx12=ros.Time.toSeconds(t1)-ros.Time.toSeconds(TX1);
          ros.log.info("rsocket::solve done "+ret+" "+tx2+" "+tx12);
        }
        catch(err){ }
        if(!ret){
          if(Score[Config['fitness']][1]!=0) respNG(conn,protocol,922); //failed to solve with fitness
          else respNG(conn,protocol,923); //failed to solve except fitness
          return;
        }
        let req=new utils_srvs.TextFilter.Request();
        let getFrame=function(id){
          return Param.hasOwnProperty(id) && Param[id]!=''? Param[id]:Config[id];
        }
        req.data=getFrame('base_frame_id')+' '+getFrame('source_frame_id')+' '+getFrame('target_frame_id');
        console.log('tf lookup:'+req.data);
        let res;
        try{
          res=await tf_lookup.call(req);
        }
        catch(err){
          ros.log.error('tf_lookup call error');
          respNG(conn,protocol,924); //failed to lookup
          return;
        }
        if(res.data.length==0){
          ros.log.error('tf_lookup returned null');
          respNG(conn,protocol,924); //failed to lookup
          return;
        }
        let tf=JSON.parse(res.data);
        if(!tf.hasOwnProperty('translation')){
          ros.log.error('tf_lookup returned but Transform');
          respNG(conn,protocol,924); //failed to lookup
          return;
        }
       ros.log.info("rsocket tf:"+res.data);
       setTimeout(()=>{
         let tfs=new geometry_msgs.TransformStamped();
          tfs.header.stamp=ros.Time.now();
          tfs.header.frame_id='';
          tfs.child_frame_id=Config.rebase_frame_id;
          tfs.transform=tf;
          pub_tf.publish(tfs);
        },500);
        let cod;
        try{
          cod=await protocol.encode([tf]);
        }
        catch(e){
          ros.log.error('r_socket::pose encode error '+e);
          respNG(conn,protocol,999);
          return;
        }
        ros.log.info("rsocket encode:"+cod);
        respOK(conn,protocol,cod);
      });
    }
    async function X3(){
      f=new std_msgs.String();
      f.data=msg.trim().replace(/\).*/g, '').replace(/.*\(/, '');  //remove "*(" ")*"
      pub_recipe.publish(f);
      console.log("("+f.data+")");
      wdt=setTimeout(function(){
        wdt=null;
        emitter.removeAllListeners('recipe');
        ros.log.warn("rsocket::recipe timeout");
        respNG(conn,protocol,931);
      },Config.recipe_timeout*1000);
      emitter.removeAllListeners('recipe');
      emitter.once('recipe',function(ret){
        clearTimeout(wdt);
        wdt=null;
        ros.log.info("rsocket::recipe done "+ret);
        if(ret) respOK(conn,protocol);
        else respNG(conn,protocol,932);
      });
    }
    async function X7(){
      let pacs=msg.trim().replace(/\).*/g, '').replace(/.*\(/, '').replace(/ +/, ' ').trim();
      let args=pacs.split(' ');
      let cmd='rostopic';
      args.unshift('-1');
      args.unshift('pub');
      args.push('std_msgs/Bool');
      args.push('True');
      let pid=await spawn(cmd,args,{stdio:['inherit','pipe','inherit']});
      pid.stdout.on('data',()=>{
        respOK(conn,protocol);
       });
      pid.on('exit',(data)=>{
        if(Number(data)!=0){
          console.log('rostopic pub error');
          respNG(conn,protocol,972);
         }
       });
     }
    async function X8(){
      let pacs=msg.trim().replace(/\).*/g, '').replace(/.*\(/, '').replace(/ +/, ' ').trim();
      let args=pacs.split(' ');
      let cmd='rosparam';
      if(args.length==1){
        args.unshift('get');
        let pid=await spawn(cmd,args,{stdio:['inherit','pipe','inherit']});
        pid.stdout.on('data',(data)=>{
            respOK(conn,protocol,data.toString().trim());
        });
        pid.on('exit',(data)=>{
          if(Number(data)!=0){
            console.log('rosparam get error');
            respNG(conn,protocol,982);
          }
        });
      }
      else if(args.length==2){
        args.unshift('set');
        console.log('rosparam set '+args[0]);
        let pid=await spawn(cmd,args,{stdio:['inherit','inherit','inherit']});
        respOK(conn,protocol,args[0]);
      }
      else respNG(conn,protocol,981);
    }
    conn.on('data',async function(data){
      buffer+=data.toString();
      ros.log.info("rsocket "+buffer+" "+data.toString());
      if(buffer.indexOf('(')*buffer.indexOf(')')<0) return;//until buffer will like "??(???)"
      if(wdt!=null){
        ros.log.warn("rsocket::Xcmd busy");
        respNG(conn,protocol,900); //busy
        return;
       }
      if(!Xable){
        respNG(conn,protocol,901);  //X command disabled
        return;
       }
      //message received
      stat_out(true);
      let msgs=buffer.split(')');
      msg=msgs.shift().trim()+')';
      do{
        if(msg.startsWith('X012')){//--------------------[X012] CLEAR|CAPT|SOLVE
          conn.x012=true;
          X0();
        }
        else if(msg.startsWith('X0')) X0();//ROVI_CLEAR
        else if(msg.startsWith('X1')) X1();//ROVI_CAPTURE
        else if(msg.startsWith('X2')) X2();//ROVI_SOLVE
        else if(msg.startsWith('X3')) X3();//ROVI_RECIPE
        else if(msg.startsWith('X7')) X7();//ROS command
        else if(msg.startsWith('X8')) X8();//ROS command
        else if(msg.startsWith('J6')){
          let j6=await protocol.decode_(msg.substr(2).trim());
          console.log("J6 "+protocol.tflib.option+' '+j6[0][0])
          if(reverse_frame_updater!=null){
            let jr=Number(j6[0][0]);
            if(protocol.tflib.option.indexOf('deg')>=0) jr*=0.017453292519943295;
            reverse_frame_updater(jr);
          }
        }
        if(msgs.length==0) break; 
        msg=msgs.shift().trim();
        if(msg.length<2) break;
        else msg+=')';
      } while(true);
      buffer=msg='';
      stat_out(false);
    });
    conn.on('close', function(){
      if(wdt!=null) clearTimeout(wdt);
      wdt=null;
      stat_out(false);
      ros.log.warn('r_socket CLOSED.');
    });
    conn.on('timeout',function(){
      emitter.removeAllListeners();
      ros.log.warn('rsocket TIMEOUT');
      respNG(conn,protocol,408);
     stat_out(false);
    });
    conn.on('error', function(err){
      ros.log.warn('Net:socket error '+err);
      stat_out(false);
    });
  }).listen(Config.port);
  setInterval(async ()=>{
    try{
      let pa=await rosNode.getParam(namespace);
      Object.assign(Param,pa);
    }
    catch(e){
    }
    if(Config.x1interlock.length>0){
      Param.x1interlock=await rosNode.getParam(Config.x1interlock);
    }
  },1000);
});
