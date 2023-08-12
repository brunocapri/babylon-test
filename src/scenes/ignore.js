
const { Scene, Axis, Space, MeshBuilder, Quaternion, Vector3, Matrix, TransformNode, KeyboardEventTypes, FollowCamera, SceneLoader, HemisphericLight } = BABYLON
const { PhysicsBody, PhysicsShapeMesh, PhysicsShapeBox, PhysicsMotionType, PhysicsRaycastResult } = BABYLON
const { GridMaterial } = BABYLON
const { HavokPlugin } = BABYLON

var createScene = async function () {
    // This creates a basic Babylon Scene object (non-mesh)
    var scene = new Scene(engine);

    // This creates a light, aiming 0,1,0 - to the sky (non-mesh)
    var light = new HemisphericLight("light", new Vector3(0, 1, 0), scene);

    // Default intensity is 1. Let's dim the light a small amount
    light.intensity = 0.7;

    const havokPlugin = await new HavokPlugin();
    scene.enablePhysics(new Vector3(0, -9.8, 0), havokPlugin);
    
    const levelImport = await SceneLoader.ImportMeshAsync("", "https://raw.githubusercontent.com/RaggarDK/Baby/baby/", "track6.babylon", scene)
    
    levelImport.meshes.forEach((mesh,index) => {
        mesh.position.y = -5
        if(index == 4) mesh.position.y = -10
        mesh.scaling.y = 0.4
        mesh.bakeCurrentTransformIntoVertices()
        mesh.material = new GridMaterial("")

        const groundBody = new PhysicsBody(
            mesh,
            PhysicsMotionType.STATIC,
            false,
            scene
        );

        const groundShape = new PhysicsShapeMesh(
            mesh,
            scene  
        );

        groundShape.material = { friction: 0.2, restitution: 0.3 };
        groundBody.shape = groundShape;
    })

    

    const curveContainer = new TransformNode("curves")
    const animations = await BABYLON.Animation.CreateFromSnippetAsync('1CG7SN#3');
    curveContainer.animations = animations;
    const accelerationCurve = animations[0]
    const skidCurve = animations[1]

    const chassisMesh = MeshBuilder.CreateBox("chassis", {width:2, height:1, depth:4})
    chassisMesh.position.y = 4
    chassisMesh.computeWorldMatrix(true)
    const chassisBody = new PhysicsBody(
        chassisMesh,
        PhysicsMotionType.DYNAMIC,
        false,
        scene
    );


    const chassisShape = new PhysicsShapeBox(
        new Vector3(0, 0, 0),
        Quaternion.Identity(),
        new Vector3(2, 1, 4),
        scene
    );



    chassisShape.material = { friction: 0.2, restitution: 0.3 };
    chassisBody.shape = chassisShape;
    chassisBody.setMassProperties({
        mass: 200,
        inertia: new Vector3(1, 1, 1),
        centerOfMass: new Vector3(0, -0.2, 0),
    });

    const physicsEngine = scene.getPhysicsEngine()

    const vehicle = new RaycastVehicle({
        chassisBody: chassisBody,
    })
    const followCamera = new FollowCamera("", new Vector3())
    followCamera.rotationOffset = 180
    followCamera.lockedTarget = chassisMesh
    scene.activeCamera = followCamera
    
    const wheelPositions = [
        new Vector3(-0.95,0,-1.8),
        new Vector3(0.95,0,-1.8),
        new Vector3(-0.95,0,1.8),
        new Vector3(0.95,0,1.8)
    ]

    const options = {
        radius: 0.5,
        directionLocal: new Vector3(0, -1, 0),
        suspensionStiffness: 10,
        suspensionRestLength: 1,
        frictionSlip: 4,
        dampingRelaxation: 2.3,
        dampingCompression: 1.4,
        maxSuspensionForce: 100000,
        rollInfluence: 0.001,
        axleLocal: new Vector3(1, 0, 0),
        chassisConnectionPointLocal: new Vector3(1, 1, 0),
        maxSuspensionTravel: 0.3
    }

    wheelPositions.forEach(positionLocal => {
        options.chassisConnectionPointLocal.copyFrom(positionLocal)
        vehicle.addWheel(options)
    })

    vehicle.addToWorld(physicsEngine)

    const wheelMesh = MeshBuilder.CreateCylinder("", {height:0.4, diameter:1, tessellation:6})
    wheelMesh.rotationQuaternion = new Quaternion()
    wheelMesh.rotate(Axis.Z, Math.PI/2, Space.LOCAL)
    wheelMesh.bakeCurrentTransformIntoVertices()

    const wheelMeshes = [
        wheelMesh.createInstance("wheel"),
        wheelMesh.createInstance("wheel"),
        wheelMesh.createInstance("wheel"),
        wheelMesh.createInstance("wheel"),
    ]

    const controls = {
        forwards:false,
        backwards:false,
        left:false,
        right:false
    }

    scene.onKeyboardObservable.add((kbInfo) => {
        switch (kbInfo.type) {
            case KeyboardEventTypes.KEYDOWN:
            if(kbInfo.event.key == 'w') controls.forwards = true
            if(kbInfo.event.key == 's') controls.backwards = true
            if(kbInfo.event.key == 'a') controls.left = true
            if(kbInfo.event.key == 'd') controls.right = true
            if(kbInfo.event.key == ' ') controls.brake = true
            break;
            case KeyboardEventTypes.KEYUP:
            if(kbInfo.event.key == 'w') controls.forwards = false
            if(kbInfo.event.key == 's') controls.backwards = false
            if(kbInfo.event.key == 'a') controls.left = false
            if(kbInfo.event.key == 'd') controls.right = false
            if(kbInfo.event.key == ' ') controls.brake = false
            break;
        }
    });

    let forwardForce = 0
    let steerValue = 0
    let steerDirection = 0
    let brakeForce = 0

    const maxSpeed = 70
    const maxForce = 1500
    const maxBrakeForce = 45
    const maxSteerValue = 0.6 
    const steeringIncrement = 0.15
    const steerRecover = 0.15



    scene.registerBeforeRender(()=>{
        forwardForce = 0
        brakeForce = 0
        steerDirection = 0
        if(controls.forwards) forwardForce = -1
        if(controls.backwards) forwardForce = 1
        if(controls.left) steerDirection = -1
        if(controls.right) steerDirection = 1
        if(controls.brake) brakeForce = maxBrakeForce

        steerValue += steerDirection*steeringIncrement
        steerValue = Math.min(Math.max(steerValue, -maxSteerValue), maxSteerValue)
        steerValue *= 1-(1-Math.abs(steerDirection))*steerRecover

        let speed = Math.abs(vehicle.currentVehicleSpeedKmHour)
        speed = Math.min(speed, maxSpeed)
        const prog = (speed/maxSpeed)*100
        const acceleration = accelerationCurve.evaluate(prog)
        const force = acceleration*forwardForce*maxForce

        const slip = skidCurve.evaluate(prog)
        const slipForce = 8-(slip*4)


        vehicle.applyEngineForce(0, 0)
        vehicle.applyEngineForce(0, 1)
        vehicle.applyEngineForce(force, 2)
        vehicle.applyEngineForce(force, 3)

        vehicle.setSteeringValue(steerValue, 2)
        vehicle.setSteeringValue(steerValue, 3)

        vehicle.setBrake(brakeForce, 0)
        vehicle.setBrake(brakeForce, 1)
        vehicle.setBrake(0, 2)
        vehicle.setBrake(0, 3)

        vehicle.wheelInfos[0].frictionSlip = slipForce
        vehicle.wheelInfos[1].frictionSlip = slipForce
        vehicle.wheelInfos[2].frictionSlip = slipForce
        vehicle.wheelInfos[3].frictionSlip = slipForce

        chassisBody.transformNode.computeWorldMatrix(true)
        
        vehicle.updateVehicle(0.016)

        for (let i = 0; i < vehicle.wheelInfos.length; i++) {
            vehicle.updateWheelTransform(i)
            const transform = vehicle.wheelInfos[i].worldTransform
            wheelMeshes[i].position.copyFrom(transform.position)
            wheelMeshes[i].rotationQuaternion.copyFrom(transform.rotationQuaternion)
        }

        //console.log(vehicle.wheelInfos[0])
    })

    return scene;
};


var directions = [
    new Vector3(1, 0, 0),
    new Vector3(0, 1, 0),
    new Vector3(0, 0, 1)
];


var calcRollingFriction_vel1 = new Vector3();
var calcRollingFriction_vel2 = new Vector3();
var calcRollingFriction_vel = new Vector3();

var updateFriction_surfNormalWS_scaled_proj = new Vector3();
var updateFriction_axle = [];
var updateFriction_forwardWS = [];
var sideFrictionStiffness2 = 1;

var castRay_rayvector = new Vector3();
var castRay_target = new Vector3();


var tmpVec1 = new Vector3();
var tmpVec2 = new Vector3();
var tmpVec3 = new Vector3();
var tmpVec4 = new Vector3();
var tmpVec5 = new Vector3();
var tmpVec6 = new Vector3();
var torque = new Vector3();

const tmpMat1 = new Matrix()

class RaycastVehicle{
    constructor(options){
        this.chassisBody = options.chassisBody;
        this.wheelInfos = [];
        this.sliding = false;
        this.world = null;
        this.indexRightAxis = typeof(options.indexRightAxis) !== 'undefined' ? options.indexRightAxis : 0;
        this.indexForwardAxis = typeof(options.indexForwardAxis) !== 'undefined' ? options.indexForwardAxis : 2;
        this.indexUpAxis = typeof(options.indexUpAxis) !== 'undefined' ? options.indexUpAxis : 1;
    }

    addWheel(options){
        options = options || {};
    
        var info = new WheelInfo(options);
        var index = this.wheelInfos.length;
        this.wheelInfos.push(info);
    
        return index;
    }

    setSteeringValue(value, wheelIndex){
        var wheel = this.wheelInfos[wheelIndex];
        wheel.steering = value;
    }

    
    applyEngineForce(value, wheelIndex){
        this.wheelInfos[wheelIndex].engineForce = value;
    }

    setBrake(brake, wheelIndex){
        this.wheelInfos[wheelIndex].brake = brake;
    }


    addToWorld(world){
        this.world = world;
    }

    getVehicleAxisWorld(axisIndex, result){
        result.set(
            axisIndex === 0 ? 1 : 0,
            axisIndex === 1 ? 1 : 0,
            axisIndex === 2 ? 1 : 0
        );
        Vector3.TransformCoordinatesToRef(result, bodyTransform(this.chassisBody, new Matrix()), result);
        return result;
    }

    updateVehicle(timeStep){
		var wheelInfos = this.wheelInfos;
        var numWheels = wheelInfos.length;
        var chassisBody = this.chassisBody;

        for (var i = 0; i < numWheels; i++) {
            this.updateWheelTransform(i);
        }
        const cVel = bodyLinearVelocity(chassisBody, new Vector3())
        const cVelLocal = Vector3.TransformNormalToRef(cVel, bodyTransform(chassisBody, new Matrix()).invert(),new Vector3())
        this.currentVehicleSpeedKmHour = cVelLocal.z

        var forwardWorld = new Vector3();
        this.getVehicleAxisWorld(this.indexForwardAxis, forwardWorld);

        if (Vector3.Dot(forwardWorld,bodyLinearVelocity(chassisBody, new Vector3())) < 0){
            //this.currentVehicleSpeedKmHour *= -1;
        }

        // simulate suspension
        for (var i = 0; i < numWheels; i++) {
            this.castRay(wheelInfos[i]);
        }

        this.updateSuspension(timeStep);

        var impulse = new Vector3();
        for (var i = 0; i < numWheels; i++) {
            //apply suspension force
            var wheel = wheelInfos[i];
            var suspensionForce = wheel.suspensionForce;
            if (suspensionForce > wheel.maxSuspensionForce) {
                suspensionForce = wheel.maxSuspensionForce;
            }
            
            impulse.copyFrom(wheel.raycastResult.hitNormalWorld).scaleInPlace(suspensionForce * timeStep)
            addImpulseAt(chassisBody, impulse, wheel.raycastResult.hitPointWorld)
        }

        this.updateFriction(timeStep);

        var hitNormalWorldScaledWithProj = new Vector3();
        var fwd  = new Vector3();
        var vel = new Vector3();
        for (i = 0; i < numWheels; i++) {
            var wheel = wheelInfos[i];
            velocityAt(chassisBody, wheel.chassisConnectionPointWorld, vel)
            // Hack to get the rotation in the correct direction
            var m = 1;
            switch(this.indexUpAxis){
            case 1:
                m = -1;
                break;
            }

            if (wheel.isInContact) {

                this.getVehicleAxisWorld(this.indexForwardAxis, fwd);
                var proj = Vector3.Dot(fwd, wheel.raycastResult.hitNormalWorld);
                hitNormalWorldScaledWithProj.copyFrom(wheel.raycastResult.hitNormalWorld).scaleInPlace(proj)

                fwd.subtractToRef(hitNormalWorldScaledWithProj, fwd);

                var proj2 = Vector3.Dot(fwd, vel);
                wheel.deltaRotation = m * proj2 * timeStep / wheel.radius;
            }

            if((wheel.sliding || !wheel.isInContact) && wheel.engineForce !== 0 && wheel.useCustomSlidingRotationalSpeed){
                // Apply custom rotation when accelerating and sliding
                wheel.deltaRotation = (wheel.engineForce > 0 ? 1 : -1) * wheel.customSlidingRotationalSpeed * timeStep;
            }

            // Lock wheels
            if(Math.abs(wheel.brake) > Math.abs(wheel.engineForce)){
                wheel.deltaRotation = 0;
            }

            wheel.rotation += wheel.deltaRotation; // Use the old value
            wheel.deltaRotation *= 0.99; // damping of rotation when not in contact
        }
    }


    updateSuspension(deltaTime) {
        var chassisBody = this.chassisBody;
        var chassisMass = bodyMass(chassisBody);
        var wheelInfos = this.wheelInfos;
        var numWheels = wheelInfos.length;

        for (var w_it = 0; w_it < numWheels; w_it++){
            var wheel = wheelInfos[w_it];

            if (wheel.isInContact){
                var force;

                // Spring
                var susp_length = wheel.suspensionRestLength;
                var current_length = wheel.suspensionLength;
                var length_diff = (susp_length - current_length);

                force = wheel.suspensionStiffness * length_diff * wheel.clippedInvContactDotSuspension;

                // Damper
                var projected_rel_vel = wheel.suspensionRelativeVelocity;
                var susp_damping;
                if (projected_rel_vel < 0) {
                    susp_damping = wheel.dampingCompression;
                } else {
                    susp_damping = wheel.dampingRelaxation;
                }
                force -= susp_damping * projected_rel_vel;

                wheel.suspensionForce = force * chassisMass;
                if (wheel.suspensionForce < 0) {
                    wheel.suspensionForce = 0;
                }
            } else {
                wheel.suspensionForce = 0;
            }
        }
    }


    removeFromWorld(world){
        world.removeBody(this.chassisBody);
        this.world = null;
    }




    castRay(wheel) {
        var rayvector = castRay_rayvector;
        var target = castRay_target;

        this.updateWheelTransformWorld(wheel);
        var chassisBody = this.chassisBody;

        var depth = -1;

        var raylen = wheel.suspensionRestLength + wheel.radius;

        rayvector.copyFrom(wheel.directionWorld).scaleInPlace(raylen)
        var source = wheel.chassisConnectionPointWorld;
        target.copyFrom(source).addInPlace(rayvector)
        var raycastResult = wheel.raycastResult;

        var param = 0;

        raycastResult.reset();
        // Turn off ray collision with the chassis temporarily
       
        // Cast ray against world
        this.world.raycastToRef(source, target, raycastResult);
     
        var object = raycastResult.body;

        wheel.raycastResult.groundObject = 0;

        if (object) {
            const distance = Vector3.Distance(source, raycastResult.hitPointWorld)
            depth = distance;
            wheel.raycastResult.hitNormalWorld.copyFrom(raycastResult.hitNormalWorld);
            wheel.isInContact = true;

            var hitDistance = distance;
            wheel.suspensionLength = hitDistance - wheel.radius;

            // clamp on max suspension travel
            var minSuspensionLength = wheel.suspensionRestLength - wheel.maxSuspensionTravel;
            var maxSuspensionLength = wheel.suspensionRestLength + wheel.maxSuspensionTravel;
            if (wheel.suspensionLength < minSuspensionLength) {
                wheel.suspensionLength = minSuspensionLength;
            }
            if (wheel.suspensionLength > maxSuspensionLength) {
                wheel.suspensionLength = maxSuspensionLength;
                wheel.raycastResult.reset();
            }

            var denominator = Vector3.Dot(wheel.raycastResult.hitNormalWorld,wheel.directionWorld);

            var chassis_velocity_at_contactPoint = new Vector3();
            velocityAt(chassisBody, wheel.raycastResult.hitPointWorld, chassis_velocity_at_contactPoint)
            var projVel = Vector3.Dot(wheel.raycastResult.hitNormalWorld, chassis_velocity_at_contactPoint );

            if (denominator >= -0.1) {
                wheel.suspensionRelativeVelocity = 0;
                wheel.clippedInvContactDotSuspension = 1 / 0.1;
            } else {
                var inv = -1 / denominator;
                wheel.suspensionRelativeVelocity = projVel * inv;
                wheel.clippedInvContactDotSuspension = inv;
            }

        } else {

            //put wheel info as in rest position
            wheel.suspensionLength = wheel.suspensionRestLength + 0 * wheel.maxSuspensionTravel;
            wheel.suspensionRelativeVelocity = 0.0;
            wheel.raycastResult.hitNormalWorld.copyFrom(wheel.directionWorld).scaleInPlace(-1)
            wheel.clippedInvContactDotSuspension = 1.0;
        }

        return depth;
    }

    updateWheelTransformWorld(wheel){
        wheel.isInContact = false;
        var chassisBody = this.chassisBody;
        const transform = bodyTransform(chassisBody, new Matrix())
        Vector3.TransformCoordinatesToRef(wheel.chassisConnectionPointLocal, transform, wheel.chassisConnectionPointWorld)
        Vector3.TransformNormalToRef(wheel.directionLocal, transform, wheel.directionWorld)
        Vector3.TransformNormalToRef(wheel.axleLocal, transform, wheel.axleWorld)
    }

    
    updateWheelTransform(wheelIndex){
        var up = tmpVec4;
        var right = tmpVec5;
        var fwd = tmpVec6;
    
        var wheel = this.wheelInfos[wheelIndex];
        this.updateWheelTransformWorld(wheel);
    
        wheel.directionLocal.scaleToRef(-1, up);
        right.copyFrom(wheel.axleLocal);
        Vector3.CrossToRef(up, right, fwd);
        fwd.normalize();
        right.normalize();
    
        // Rotate around steering over the wheelAxle
        var steering = wheel.steering;
        var steeringOrn = new Quaternion();
        Quaternion.RotationAxisToRef(up, steering, steeringOrn)
    
        var rotatingOrn = new Quaternion();
        Quaternion.RotationAxisToRef(right, wheel.rotation, rotatingOrn)
    
        // World rotation of the wheel
        var q = wheel.worldTransform.rotationQuaternion;
        
        bodyOrientation(this.chassisBody, new Quaternion()).multiplyToRef(steeringOrn, q);
        q.multiplyToRef(rotatingOrn, q);
    
        q.normalize();
    
        // world position of the wheel
        var p = wheel.worldTransform.position;
        p.copyFrom(wheel.directionWorld)
        p.scaleToRef(wheel.suspensionLength, p);
        p.addToRef(wheel.chassisConnectionPointWorld, p);

        wheel.worldTransform.computeWorldMatrix(true)
    }

    getWheelTransformWorld(wheelIndex) {
        return this.wheelInfos[wheelIndex].worldTransform;
    }


    updateFriction(timeStep){
        var surfNormalWS_scaled_proj = updateFriction_surfNormalWS_scaled_proj;

        //calculate the impulse, so that the wheels don't move sidewards
        var wheelInfos = this.wheelInfos;
        var numWheels = wheelInfos.length;
        var chassisBody = this.chassisBody;
        var forwardWS = updateFriction_forwardWS;
        var axle = updateFriction_axle;

        var numWheelsOnGround = 0;

        for (var i = 0; i < numWheels; i++) {
            var wheel = wheelInfos[i];

            var groundObject = wheel.raycastResult.body;
            if (groundObject){
                numWheelsOnGround++;
            }

            wheel.sideImpulse = 0;
            wheel.forwardImpulse = 0;
            if(!forwardWS[i]){
                forwardWS[i] = new Vector3();
            }
            if(!axle[i]){
                axle[i] = new Vector3();
            }
        }
        
        for (var i = 0; i < numWheels; i++){
            var wheel = wheelInfos[i];
    
            var groundObject = wheel.raycastResult.body;
    
            if (groundObject) {

                var axlei = axle[i];
                var wheelTrans = this.getWheelTransformWorld(i);
    
                // Get world axle
                Vector3.TransformNormalToRef(directions[this.indexRightAxis], wheelTrans.getWorldMatrix(), axlei);
        
                var surfNormalWS = wheel.raycastResult.hitNormalWorld;
                var proj = Vector3.Dot(axlei, surfNormalWS);
                
                surfNormalWS.scaleToRef(proj, surfNormalWS_scaled_proj);
                axlei.subtractToRef(surfNormalWS_scaled_proj, axlei);
                axlei.normalize();
               
                Vector3.CrossToRef(surfNormalWS, axlei, forwardWS[i]);
                forwardWS[i].normalize();
                //console.log(forwardWS[i])

                
                
                wheel.sideImpulse = resolveSingleBilateral(
                    chassisBody,
                    wheel.raycastResult.hitPointWorld,
                    groundObject,
                    wheel.raycastResult.hitPointWorld,
                    axlei
                );
                //if(i == 0) console.log(wheel.sideImpulse)
                wheel.sideImpulse *= sideFrictionStiffness2;
            }
        }

        var sideFactor = 1;
        var fwdFactor = 0.5;

        this.sliding = false;
        for (var i = 0; i < numWheels; i++) {
            var wheel = wheelInfos[i];
            var groundObject = wheel.raycastResult.body;

            var rollingFriction = 0;

            wheel.slipInfo = 1;
            if (groundObject) {
                var defaultRollingFrictionImpulse = 0;
                var maxImpulse = wheel.brake ? wheel.brake : defaultRollingFrictionImpulse;

                // btWheelContactPoint contactPt(chassisBody,groundObject,wheelInfraycastInfo.hitPointWorld,forwardWS[wheel],maxImpulse);
                // rollingFriction = calcRollingFriction(contactPt);
                rollingFriction = calcRollingFriction(chassisBody, groundObject, wheel.raycastResult.hitPointWorld, forwardWS[i], maxImpulse);

                rollingFriction += wheel.engineForce * timeStep;

                // rollingFriction = 0;
                var factor = maxImpulse / rollingFriction;
                wheel.slipInfo *= factor;
            }

            //switch between active rolling (throttle), braking and non-active rolling friction (nthrottle/break)

            wheel.forwardImpulse = 0;
            wheel.skidInfo = 1;

            if (groundObject) {
                wheel.skidInfo = 1;

                var maximp = wheel.suspensionForce * timeStep * wheel.frictionSlip;
                var maximpSide = maximp;

                var maximpSquared = maximp * maximpSide;

                wheel.forwardImpulse = rollingFriction;//wheelInfo.engineForce* timeStep;

                var x = wheel.forwardImpulse * fwdFactor;
                var y = wheel.sideImpulse * sideFactor;

                var impulseSquared = x * x + y * y;

                wheel.sliding = false;
                if (impulseSquared > maximpSquared) {
                    this.sliding = true;
                    wheel.sliding = true;

                    var factor = maximp / Math.sqrt(impulseSquared);

                    wheel.skidInfo *= factor;
                }
            }
        }

        if (this.sliding) {
            for (var i = 0; i < numWheels; i++) {
                var wheel = wheelInfos[i];
                if (wheel.sideImpulse !== 0) {
                    if (wheel.skidInfo < 1){
                        wheel.forwardImpulse *= wheel.skidInfo;
                        wheel.sideImpulse *= wheel.skidInfo;
                    }
                }
            }
        }

        
        // apply the impulses
        for (var i = 0; i < numWheels; i++) {
            var wheel = wheelInfos[i];
    
            var rel_pos = new Vector3();
            wheel.raycastResult.hitPointWorld.subtractToRef(bodyPosition(chassisBody, new Vector3()), rel_pos);
           
            if (wheel.forwardImpulse !== 0) {
                var impulse = new Vector3();
                impulse.copyFrom(forwardWS[i]).scaleInPlace(wheel.forwardImpulse)
                addImpulseAt(chassisBody, impulse, wheel.raycastResult.hitPointWorld)
            
            }
    
            if (wheel.sideImpulse !== 0){
                var groundObject = wheel.raycastResult.body;
    
                var rel_pos2 = new Vector3();
                wheel.raycastResult.hitPointWorld.subtractToRef(bodyPosition(groundObject, new Vector3()), rel_pos2);
                var sideImp = new Vector3();
                sideImp.copyFrom(axle[i]).scaleInPlace(wheel.sideImpulse)
    
                Vector3.TransformNormalToRef(rel_pos, bodyTransform(chassisBody, new Matrix()).invert(), rel_pos);
                rel_pos['xyz'[this.indexUpAxis]] *= wheel.rollInfluence;
                
                Vector3.TransformNormalToRef(rel_pos, bodyTransform(chassisBody, new Matrix()), rel_pos);
                addImpulseAt(chassisBody, sideImp, bodyPosition(chassisBody, new Vector3()).add(rel_pos));
			
                sideImp.scaleToRef(-1, sideImp);
                addImpulseAt(groundObject, sideImp, wheel.raycastResult.hitPointWorld);
                
            }
        }
        
    }

}

function calcRollingFriction(body0, body1, frictionPosWorld, frictionDirectionWorld, maxImpulse) {
    var j1 = 0;
    var contactPosWorld = frictionPosWorld;

    var vel1 = calcRollingFriction_vel1;
    var vel2 = calcRollingFriction_vel2;
    var vel = calcRollingFriction_vel;
   
    velocityAt(body0, contactPosWorld, vel1);
    velocityAt(body1, contactPosWorld, vel2);
    vel1.subtractToRef(vel2, vel);

    var vrel = Vector3.Dot(frictionDirectionWorld, vel);

    var denom0 = computeImpulseDenominator(body0, frictionPosWorld, frictionDirectionWorld);
    var denom1 = computeImpulseDenominator(body1, frictionPosWorld, frictionDirectionWorld);
    var relaxation = 1;
    var jacDiagABInv = relaxation / (denom0 + denom1);

    // calculate j that moves us to zero relative velocity
    j1 = -vrel * jacDiagABInv;

    if (maxImpulse < j1) {
        j1 = maxImpulse;
    }
    if (j1 < -maxImpulse) {
        j1 = -maxImpulse;
    }

    return j1;
}



var computeImpulseDenominator_r0 = new Vector3();
var computeImpulseDenominator_c0 = new Vector3();
var computeImpulseDenominator_vec = new Vector3();
var computeImpulseDenominator_m = new Vector3();

function computeImpulseDenominator(body, pos, normal) {
    var r0 = computeImpulseDenominator_r0;
    var c0 = computeImpulseDenominator_c0;
    var vec = computeImpulseDenominator_vec;
    var m = computeImpulseDenominator_m;

    pos.subtractToRef(bodyPosition(body, new Vector3()), r0);
    Vector3.CrossToRef(r0, normal, c0);
    bodyInertiaWorld(body, new Vector3()).multiplyToRef(c0, m)
    

    Vector3.CrossToRef(m, r0, vec);

    return bodyInvMass(body) + Vector3.Dot(normal, vec);
}



var resolveSingleBilateral_vel1 = new Vector3();
var resolveSingleBilateral_vel2 = new Vector3();
var resolveSingleBilateral_vel = new Vector3();



function resolveSingleBilateral(body1, pos1, body2, pos2, normal){
    var normalLenSqr = normal.lengthSquared()
    if (normalLenSqr > 1.1){
        return 0; // no impulse
    }
    var vel1 = resolveSingleBilateral_vel1;
    var vel2 = resolveSingleBilateral_vel2;
    var vel = resolveSingleBilateral_vel;
    velocityAt(body1, pos1, vel1)
    velocityAt(body2, pos2, vel2)
    
    vel1.subtractToRef(vel2, vel);

    var rel_vel = Vector3.Dot(normal, vel);

    var contactDamping = 0.1;
    var massTerm = 1 / (bodyInvMass(body1) + bodyInvMass(body2));
    var impulse = - contactDamping * rel_vel * massTerm;

    return impulse;
}



var chassis_velocity_at_contactPoint = new Vector3();
var relpos = new Vector3();
var chassis_velocity_at_contactPoint = new Vector3();



const Utilsdefaults = (options, defaults) => {
    options = options || {};

    for(var key in defaults){
        if(!(key in options)){
            options[key] = defaults[key];
        }
    }

    return options;
};





class WheelInfo{
    constructor(options){
        options = Utilsdefaults(options, {
            chassisConnectionPointLocal: new Vector3(),
            chassisConnectionPointWorld: new Vector3(),
            directionLocal: new Vector3(),
            directionWorld: new Vector3(),
            axleLocal: new Vector3(),
            axleWorld: new Vector3(),
            suspensionRestLength: 5,
            suspensionMaxLength: 10,
            radius: 1,
            suspensionStiffness: 10,
            dampingCompression: 0,
            dampingRelaxation: 0,
            frictionSlip: 100,
            steering: 0,
            rotation: 0,
            deltaRotation: 0,
            rollInfluence: 0.01,
            maxSuspensionForce: Number.MAX_VALUE,
            isFrontWheel: true,
            clippedInvContactDotSuspension: 1,
            suspensionRelativeVelocity: 0,
            suspensionForce: 0,
            skidInfo: 0,
            suspensionLength: 0,
            maxSuspensionTravel: 1,
            useCustomSlidingRotationalSpeed: false,
            customSlidingRotationalSpeed: -0.1
        });

        this.maxSuspensionTravel = options.maxSuspensionTravel;
        this.customSlidingRotationalSpeed = options.customSlidingRotationalSpeed;
        this.useCustomSlidingRotationalSpeed = options.useCustomSlidingRotationalSpeed;
        this.sliding = false;
        this.chassisConnectionPointLocal = options.chassisConnectionPointLocal.clone();
        this.chassisConnectionPointWorld = options.chassisConnectionPointLocal.clone();
        this.directionLocal = options.directionLocal.clone();
        this.directionWorld = options.directionLocal.clone();
        this.axleLocal = options.axleLocal.clone();
        this.axleWorld = options.axleLocal.clone();
        this.suspensionRestLength = options.suspensionRestLength;
        this.suspensionMaxLength = options.suspensionMaxLength;
        this.radius = options.radius;
        this.suspensionStiffness = options.suspensionStiffness;
        this.dampingCompression = options.dampingCompression;
        this.dampingRelaxation = options.dampingRelaxation;
        this.frictionSlip = options.frictionSlip;
        this.steering = 0;
        this.rotation = 0;
        this.deltaRotation = 0;
        this.rollInfluence = options.rollInfluence;
        this.maxSuspensionForce = options.maxSuspensionForce;
        this.engineForce = 0;
        this.brake = 0;
        this.isFrontWheel = options.isFrontWheel;
        this.clippedInvContactDotSuspension = 1;
        this.suspensionRelativeVelocity = 0;
        this.suspensionForce = 0;
        this.skidInfo = 0;
        this.suspensionLength = 0;
        this.sideImpulse = 0;
        this.forwardImpulse = 0;
        this.raycastResult = new PhysicsRaycastResult();
        this.raycastResult.directionWorld = new Vector3()
        this.worldTransform = new TransformNode("")
        this.worldTransform.rotationQuaternion = new Quaternion()
        this.isInContact = false;
    }

    updateWheel(chassis){
        var raycastResult = this.raycastResult;
    
        if (this.isInContact){
            var project= Vector3.Dot(raycastResult.hitNormalWorld, raycastResult.directionWorld);
            raycastResult.hitPointWorld.subtractToRef( bodyPosition(chassis, new Vector3()), relpos);
            velocityAt(chassis, relpos, chassis_velocity_at_contactPoint);
           // velocityAt(chassis, raycastResult.hitPointWorld, relpos);
            var projVel = Vector3.Dot(raycastResult.hitNormalWorld, chassis_velocity_at_contactPoint );
            if (project >= -0.1) {
                this.suspensionRelativeVelocity = 0.0;
                this.clippedInvContactDotSuspension = 1.0 / 0.1;
            } else {
                var inv = -1 / project;
                this.suspensionRelativeVelocity = projVel * inv;
                this.clippedInvContactDotSuspension = inv;
            }
    
        } else {
            // Not in contact : position wheel in a nice (rest length) position
            raycastResult.suspensionLength = this.suspensionRestLength;
            this.suspensionRelativeVelocity = 0.0;
            raycastResult.hitNormalWorld.copyFrom(raycastResult.directionWorld).scaleInPlace(-1)
            this.clippedInvContactDotSuspension = 1.0;
        }
    }
}




const tmpVel2 = new Vector3()
const velocityAt = (body, pos, res) => {
    body.getObjectCenterWorldToRef(tmpVel2)
    pos.subtractToRef(tmpVel2, res)
    body.getAngularVelocityToRef(tmpVel2)
    Vector3.CrossToRef(tmpVel2, res, res)
    body.getLinearVelocityToRef(tmpVel2)
    res.addInPlace(tmpVel2)
    return res;
}

const bodyPosition = (body, res) => {
    body.getObjectCenterWorldToRef(res)
    return res
}

const bodyLinearVelocity = (body, res) => {
    body.getLinearVelocityToRef(res)
    return res
}

const bodyAngularVelocity = (body, res) => {
    body.getAngularVelocityToRef(res)
    return res
}

const bodyTransform = (body, res) => {
    res.copyFrom(body.transformNode.getWorldMatrix())
    return res
}

const addImpulseAt = (body, impulse, point)=>{
    body.applyImpulse(
        impulse,
        point
    )
}

const addForceAt = (body, force, point)=>{
    body.applyForce(
        force,
        point
    )
}


const bodyOrientation = (body, res) => {
    res.copyFrom(body.transformNode.rotationQuaternion)
    return res
}

const bodyMass = (body) => {
    return body.getMassProperties().mass
}

const bodyInvMass = (body) => {
    const mass = body.getMassProperties().mass
    return mass > 0 ? 1.0 / mass : 0
}

const bodyInertiaWorld = (body, res) => {
    const prop = body.getMassProperties()
    res.copyFrom(prop.inertia)
    Vector3.TransformNormalToRef(res, body.transformNode.getWorldMatrix(), res)
    //console.log(res)
    res.x = res.x > 0 ? 1.0 / res.x : 0
    res.y = res.y > 0 ? 1.0 / res.y : 0
    res.z = res.z > 0 ? 1.0 / res.z : 0
    return res
}