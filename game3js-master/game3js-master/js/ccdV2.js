var CCDsys = function() {

	// states
    this.thetas = [];
  	this.axes = [];
  	this.joints = [];
  
  	this.target = new THREE.Vector3();

	// FK function set elsewhere
	this.setFK = function( fkEval ) {

		this.fk = fkEval;

	}  

  
  	this.CCD_axis = function(axis, id, angleLo, angleHi) {

    	this.axis = axis.clone();
    	this.jointid = id;
		var thetaLo = angleLo === undefined? -1e4 : angleLo; // default: no limits
		var thetaHi = angleHi === undefined? 1e4  : angleHi;
    	this.limits = new THREE.Vector2(thetaLo, thetaHi); 

  };

	this.update = function() {

		var end = new THREE.Vector3();
    	var base = new THREE.Vector3();

		var theta = this.thetas;
		var axes = this.axes;
		var target = this.target;
    
    	// e.g., njoints = 2;
    	// jointid: 0,0,1
		var njoints = axes[axes.length - 1].jointid + 1;
		var joints = [];
		for (var i = 0; i <= njoints; i++) joints[i] = new THREE.Vector3();

		this.fk(theta, joints);
		end.copy(joints[joints.length - 1]);

		// convergence
		var eps = 1e-1;
		var MAXITER = 20;

		var t_target = new THREE.Vector3();
		var t_end = new THREE.Vector3();
		var tmpV = new THREE.Vector3();

		// iteration

		for (var iter = 0; iter < MAXITER; iter++) {
		  for (var i = axes.length - 1; i >= 0; i--) {
			base.copy(joints[axes[i].jointid]);

			// this part is quite different from the C counterpart
			var axis = axes[i].axis.clone();
			for (var j = i - 1; j >= 0; j--)
			  axis.applyMatrix4(new THREE.Matrix4().makeRotationAxis(axes[j].axis, theta[j]));

			// after this manipulation,
			// axis become world coordinate

			tmpV.subVectors(target, base);
			tmpV = proj2plane(tmpV, axis);
			t_target.copy(tmpV.normalize());

			tmpV.subVectors(end, base);
			tmpV = proj2plane(tmpV, axis);
			t_end.copy(tmpV.normalize());

			var dotV = t_end.dot(t_target);
			var angle = Math.acos(CLAMP(dotV, -1, 1));
			tmpV.crossVectors(t_end, t_target);
			var sign = (tmpV.dot(axis) > 0) ? 1 : -1;
			theta[i] += sign * angle;

			// joint limit [-2.4, -0.1]
			theta[i] = CLAMP(theta[i], axes[i].limits.x, axes[i].limits.y)

			this.fk(theta, joints);
			end.copy(joints[joints.length - 1]);

			if (end.distanceTo(target) < eps) {
			  return 1;
			}
		  }
	  
		}


		if (iter < MAXITER)
		  return 1;
		else {
		  console.log("do not converge");
		  return 0;
		}
	}


	/////////////////////////////////
	/// HELPER FUNCTIONS
	// p: the vector to be projected
	// n: the normal defining the projection plane (unit vector)
  	// clarification: call by reference/pointer or call-by-value
  	function proj2plane(p, n) {
    	return p.clone().projectOnPlane(n);
  	}
	
	function CLAMP(x, xlo, xhi) {
		if (x < xlo)
		  return xlo;
		if (x > xhi)
		  return xhi;
		return x;
  	}


}


////////////////////////////////////
var scene, renderer, camera, controls;
var link1, link2;

var target = new THREE.Vector3();
var xx = 0;
var sign = 1;

var ccdSys;

init();
animate();

////////////////////////////////////////////////////////
// forward kinematics
function fk( theta, joints ) {
	
  joints[0] = new THREE.Vector3(0, 0, 0);

  var m = new THREE.Matrix4();
  m.makeRotationY(theta[0]);
  m.multiply(new THREE.Matrix4().makeTranslation(60, 0, 0));
  var localzero = new THREE.Vector3(0, 0, 0);
  localzero.applyMatrix4(m);
  joints[1].copy(localzero);

  m.multiply(new THREE.Matrix4().makeRotationY(theta[1]));
  m.multiply(new THREE.Matrix4().makeTranslation(90, 0, 0));
  localzero.set(0, 0, 0);
  localzero.applyMatrix4(m);
  joints[2].copy(localzero);

}


////////////////////////////////////////////////////////////////
function init() {

  renderer = new THREE.WebGLRenderer({
    antialias: true
  });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColor(0x888888);

  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 10000);
  camera.position.y = 160;
  camera.position.z = 400;
  camera.lookAt(new THREE.Vector3(0, 0, 0));
  document.body.appendChild(renderer.domElement);
  controls = new THREE.OrbitControls(camera, renderer.domElement);

  var gridXZ = new THREE.GridHelper(100, 10, 'red', 'white');
  scene.add(gridXZ);

  window.addEventListener('resize', onWindowResize, false);
 
  ////////// GEOMETRIC MODEL  //////////////////////////////
  link1 = makeLink(60);
  scene.add(link1);
  link2 = makeLink(90);
  link1.add(link2);
  link2.position.set(60, 0, 0);

  // base
  var cyl_geom = new THREE.CylinderGeometry(10, 10, 6, 32);
  var cyl_mat = new THREE.MeshBasicMaterial({
    color: 0xff2211
  });
  var base = new THREE.Mesh(cyl_geom, cyl_mat);
  scene.add(base);

 	///////// KINEMATIC MODEL  ////////////////////////////////////
	ccdSys = new CCDsys ();
	ccdSys.thetas.push (0,0);  // initial values of thetas
  for (var i = 0; i < 3; i++)
    ccdSys.joints.push ( new THREE.Vector3() );

  // setting ccd_box	
  ccdSys.setFK (fk);
  
  ccdSys.axes.push ( new ccdSys.CCD_axis(new THREE.Vector3(0, 1, 0), 0) );
  ccdSys.axes.push ( new ccdSys.CCD_axis(new THREE.Vector3(0, 1, 0), 1, -3.1, -0.01) );
	
}

function makeLink(length) {
  var oneLink = new THREE.Object3D();
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(length, 10, 10), new THREE.MeshNormalMaterial());
  oneLink.add(mesh);
  mesh.position.set(length / 2, 0, 0);
  return oneLink;
}

function onWindowResize() {
  var width = window.innerWidth;
  var height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

function animate() {
  if (Math.abs(xx) > 100)
    sign *= -1;
  xx += sign * 5;

  requestAnimationFrame(animate);

  // setting target
	target.set(xx, 0, -80);

  ccdSys.target.copy (target); 

  update();
  render();
}

function update() {

  controls.update();

  ccdSys.update ();

}


function render() {
  console.log (ccdSys.thetas[0] + ',' + ccdSys.thetas[1]);
  link1.rotation.y = ccdSys.thetas[0];  //theta1;
  link2.rotation.y = ccdSys.thetas[1];  //theta2;

  renderer.render(scene, camera);
}
