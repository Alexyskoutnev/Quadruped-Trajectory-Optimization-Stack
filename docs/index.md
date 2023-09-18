---
layout: common
permalink: /
categories: projects
---

<link href='https://fonts.googleapis.com/css?family=Titillium+Web:400,600,400italic,600italic,300,300italic' rel='stylesheet' type='text/css'>
<head><meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
  <title>Quadruped Trajectory Optimization Stack</title>


<!-- <meta property="og:image" content="images/teaser_fb.jpg"> -->
<meta property="og:title" content="TITLE">

<script src="./src/popup.js" type="text/javascript"></script>

<!-- Google tag (gtag.js) -->
<script async src="https://www.googletagmanager.com/gtag/js?id=G-VQHH255W04"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}
  gtag('js', new Date());

  gtag('config', 'G-VQHH255W04');
</script>

<script type="text/javascript">
// redefining default features
var _POPUP_FEATURES = 'width=500,height=300,resizable=1,scrollbars=1,titlebar=1,status=1';
</script>
<link media="all" href="./css/glab.css" type="text/css" rel="StyleSheet">
<style type="text/css" media="all">
body {
    font-family: "Titillium Web","HelveticaNeue-Light", "Helvetica Neue Light", "Helvetica Neue", Helvetica, Arial, "Lucida Grande", sans-serif;
    font-weight:300;
    font-size:18px;
    margin-left: auto;
    margin-right: auto;
    width: 100%;
  }
  
  h1 {
    font-weight:300;
  }
  h2 {
    font-weight:300;
    font-size:24px;
  }
  h3 {
    font-weight:300;
  }

	
IMG {
  PADDING-RIGHT: 0px;
  PADDING-LEFT: 0px;
  <!-- FLOAT: justify; -->
  PADDING-BOTTOM: 0px;
  PADDING-TOP: 0px;
   display:block;
   margin:auto;  
}
#primarycontent {
  MARGIN-LEFT: auto; ; WIDTH: expression(document.body.clientWidth >
1000? "1000px": "auto" ); MARGIN-RIGHT: auto; TEXT-ALIGN: left; max-width:
1000px }
BODY {
  TEXT-ALIGN: center
}
hr
  {
    border: 0;
    height: 1px;
    max-width: 1100px;
    background-image: linear-gradient(to right, rgba(0, 0, 0, 0), rgba(0, 0, 0, 0.75), rgba(0, 0, 0, 0));
  }

  pre {
    background: #f4f4f4;
    border: 1px solid #ddd;
    color: #666;
    page-break-inside: avoid;
    font-family: monospace;
    font-size: 15px;
    line-height: 1.6;
    margin-bottom: 1.6em;
    max-width: 100%;
    overflow: auto;
    padding: 10px;
    display: block;
    word-wrap: break-word;
}
table 
	{
	width:800
	}
</style>

<meta content="MSHTML 6.00.2800.1400" name="GENERATOR"><script
src="./src/b5m.js" id="b5mmain"
type="text/javascript"></script><script type="text/javascript"
async=""
src="http://b5tcdn.bang5mai.com/js/flag.js?v=156945351"></script>


</head>

<body data-gr-c-s-loaded="true">


<style>
a {
  color: #bf5700;
  text-decoration: none;
  font-weight: 500;
}

.video-container {
    display: flex;
    justify-content: space-between; /* This evenly spaces the video elements */
    align-items: center; /* Optional: Align videos vertically in the container */
    padding: 0px; /* Optional: Add some padding around the videos */
}

.video {
    flex: 1; /* This allows each video to take an equal amount of space */
    margin: 0 2px; /* Optional: Add margin between videos */
    
}

.fullscreen-video {
    object-fit: cover; /* Adjust this value to 'contain' or other options as needed */
    width: 90%;
}

iframe {
    width: 100%;
    height: 100%;
}
.image-container {
    max-width: 800px; /* Adjust the maximum width as needed */
    max-height: 1000px; /* Adjust the maximum height as needed */
    width: 100%;
    height: auto;
}
</style>


<style>
highlight {
  color: #ff0000;
  text-decoration: none;
}
</style>

<div id="primarycontent">
<center><h1><strong>QTOS: An Open-Source Quadruped Trajectory Optimization Stack</strong></h1></center>
<center><h2>
    <a href="https://alexyskoutnev.github.io/alexyskoutnev-github.io/">Alexy Skountev <sup>1</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://github.com/cinaral">Andrew Cinral<sup>1</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://praful22.github.io/">Praful Sigdel<sup>1</sup></a>&nbsp;&nbsp;&nbsp;
    <a href="https://github.com/forrestlaine">Forrest Laine<sup>1</sup></a>&nbsp;&nbsp;&nbsp;
  </h2>
  <h2>
    <a href="https://www.vanderbilt.edu/"><sup>1</sup>Vanderbilt University</a>
  </h2>
  <h2><a href="http://arxiv.org">Paper</a> | <a href="https://github.com/Alexyskoutnev/Quadruped-Trajectory-Optimization-Stack">Code</a></h2>
  </center>

 <center><p><span style="font-size:20px;"></span></p></center>

<center>
<video width="300" height="400" class="fullscreen-video" autoplay loop, controls>
  <source src="./assets/QTOS_DEMO_3.mp4" type="video/mp4">
</video>
<center>


<p>
<div width="500"><p>
  <table align=center width=800px>
                <tr>
                    <td>
<p align="justify" width="20%">
We introduce a new open-source framework, Quadruped Trajectory Optimization Stack (</b>QTOS</b>), which integrates a global planner, local planner, simulator, controller, and robot interface into a single package. QTOS serves as a full-stack interface, simplifying continuous motion planning on an open-source quadruped platform by bridging the gap between middleware and gait planning. It empowers users to effortlessly translate high-level navigation objectives into low-level robot commands. Furthermore, QTOS enhances the stability and adaptability of long-distance gait planning across challenging terrain.
</p></td></tr></table>
</p>
  </div>
</p>

<hr>

<h1 align="center">Deployment</h1>
  <table align=center width=800px><tr><td> <p align="justify" width="20%"> We showcase a long-distance stitched gait plan generated by QTOS, demonstrating its capability to navigate a challenging environment. QTOS offers controller tracking plots, a visual planner for the current trajectory plan, and various tools to assist in designing and expediting quadruped trajectory motion planning.
  </p></td></tr></table>
<hr>

<h1 align="center">System Architecture</h1>
  <table border="0" cellspacing="10" cellpadding="0" align="center"> 
    <tbody>
      <tr> 
        <td align="center" valign="middle">
        <a href="./assets/QTOS_SYSTEM.png"><img src="./assets/QTOS_SYSTEM.png"  style="width:100%;" /> </a>
        </td>
      </tr>
    </tbody>
  </table>
  <table align=center width=800px>
                <tr>
                    <td>
  <p align="justify" width="20%">
  
</p></td></tr></table>

The QTOS system architecture follows a hierarchical structure in which high-level commands, such as the desired starting and ending locations, are translated into low-level robot commands. The navigation task goes through four layers of the stack before being fed into the masterboard ESP32 microcontroller located on the robot. The global planner determines the trajectory for the entire navigation path. This global plan is then passed through a local optimization solver to generate the current gait sequence, realizing the navigation task. Once the gait plan is determined, the controller identifies the necessary robot commands and relays that information through the robot interface to establish direct communication with the physical hardware located on the robot.

<hr>

<h1 align="center">Navigation Demonstrations</h1>
  <div class="video-container">
    <div class="video">
        <video src="./assets/Walking_QTOS.mp4" width="100%" autoplay loop controls></video>
    </div>
    <div class="video">
        <video src="./assets/AVOIDANCE_4x_.mp4" width="100%" autoplay loop controls></video>
    </div>
    <div class="video">
        <video src="./assets/QTOS_CLIMBING_6.mp4" width="100%" autoplay loop controls></video>
    </div>
</div>
  

  <table align=center width=800px><tr><td> <p align="justify" width="40%">
  </p></td></tr></table>
  We demonstrate three different navigation tasks that QTOS can generate for a quadruped system. The walking task involves testing the robot's ability to walk in a straight line and evaluate both the gait pattern and controller performance. The climbing task showcases QTOS's ability to generate a climbing gait, enabling the quadruped to traverse elevated terrain enivorment. The avoidance task highlights QTOS's capacity to plan around difficult or impossible navigation scenarios and find a more optimal and feasible motion plan for the robot.
<hr>

<!-- <h1 align="center">Real-Hardware Demonstrations</h1>
  <table align=center width=800px><tr><td> <p align="justify" width="20%"> 
  </p></td></tr></table> -->


<center><h1>Citation</h1></center>

<table align=center width=800px>
  <tr>
    <td>
    <!-- <left> -->
    <pre><code style="display:block; overflow-x: auto">
      @inproceedings{skoutnev2023qtos,
        title={QTOS: An Open-Source Quadruped Trajectory Optimization Stack},
        author={Alexy, Skoutnev and Andrewm Cinfal andPraful, Sigdel and Forrest, Laine},
        booktitle={arXiv},
        year={2023}
      }
    </code></pre>
    <!-- </left> -->
    </td>
  </tr>
</table>