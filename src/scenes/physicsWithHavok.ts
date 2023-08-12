import { Engine } from "@babylonjs/core/Engines/engine";
import { Scene } from "@babylonjs/core/scene";
import { ArcRotateCamera } from "@babylonjs/core/Cameras/arcRotateCamera";
import { Quaternion, Vector3 } from "@babylonjs/core/Maths/math.vector";
import { HemisphericLight } from "@babylonjs/core/Lights/hemisphericLight";
import { CreateGround } from "@babylonjs/core/Meshes/Builders/groundBuilder";
import "@babylonjs/core/Physics/physicsEngineComponent";

// If you don't need the standard material you will still need to import it since the scene requires it.
import "@babylonjs/core/Materials/standardMaterial";
import { CreateSceneClass } from "../createScene";
import { havokModule } from "../externals/havok";
import { PhysicsShapeBox } from "@babylonjs/core/Physics/v2/physicsShape";
import { PhysicsBody } from "@babylonjs/core/Physics/v2/physicsBody";
import { PhysicsMotionType } from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin";
import { HavokPlugin } from "@babylonjs/core/Physics/v2/Plugins/havokPlugin";
import { Color3, CreateBox, GizmoManager, Mesh, MeshBuilder, PhysicsViewer, Ray, RayHelper, StandardMaterial } from "@babylonjs/core";


class PhysicsSceneWithHavok implements CreateSceneClass {
    preTasks = [havokModule];

    createScene = async (engine: Engine, canvas: HTMLCanvasElement): Promise<Scene> => {
        // This creates a basic Babylon Scene object (non-mesh)
        const scene = new Scene(engine);

        
        // This creates and positions a free camera (non-mesh)
        const camera = new ArcRotateCamera("my first camera", 0, Math.PI / 3, 50, new Vector3(0, 0, 0), scene);

        // This targets the camera to scene origin
        camera.setTarget(Vector3.Zero());

        // This attaches the camera to the canvas
        camera.attachControl(canvas, true);

        // This creates a light, aiming 0,1,0 - to the sky (non-mesh)
        const light = new HemisphericLight("light", new Vector3(0, 1, 0), scene);

        // Default intensity is 1. Let's dim the light a small amount
        light.intensity = 0.7;
        
        scene.enablePhysics(null, new HavokPlugin(true, await havokModule));
        

        //  Configure the spaceship body
        const spaceshipShape = new PhysicsShapeBox(new Vector3(0, 0, 0),
            Quaternion.Identity(),
            new Vector3(2, 1, 4),
            scene
        );
        const spaceshipMesh = CreateBox("spaceship", { width:2, height:1, depth:4 }, scene);
        spaceshipMesh.position.y = 4;
        const spaceshipBody = new PhysicsBody(spaceshipMesh, PhysicsMotionType.DYNAMIC, false, scene);
        spaceshipShape.material = { friction: 0.2, restitution: 0.6 };
        spaceshipBody.shape = spaceshipShape;
        spaceshipBody.setMassProperties({ mass: 200, inertia: new Vector3(1, 1, 1), centerOfMass: new Vector3(0, -0.9, 0) });
        
    
        //Configure the ground
        const ground = CreateGround("ground", { width: 1000, height: 1000 }, scene);
        const groundMaterial = new StandardMaterial("matBox", scene);
        groundMaterial.diffuseColor = new Color3(0.4, 0.1, 0.1);
        ground.material = groundMaterial
        const groundShape = new PhysicsShapeBox(new Vector3(0, 0, 0), Quaternion.Identity(), new Vector3(10000, 0.1, 1000), scene);
        const groundBody = new PhysicsBody(ground, PhysicsMotionType.STATIC, false, scene);
        groundShape.material = { friction: 0.2, restitution: 0.8 };
        groundBody.shape = groundShape;
        groundBody.setMassProperties({ mass: 0 });
        groundShape.filterMembershipMask = 2;

    
        // Create Wheels
        const wheels = [
            new Wheel("lf", new Vector3(-0.95,0,1.8), scene,  spaceshipMesh, spaceshipBody),
            new Wheel("rf", new Vector3(0.95,0,1.8), scene,  spaceshipMesh, spaceshipBody),
            new Wheel("lb", new Vector3(-0.95,0,-1.8), scene,  spaceshipMesh, spaceshipBody),
            new Wheel("rb", new Vector3(0.95,0,-1.8), scene,  spaceshipMesh, spaceshipBody),
        ]

        
        
        scene.registerBeforeRender(() => {
            const dt = engine.getDeltaTime() / 1000;
            wheels.forEach((w) => {
                w.onUpdate(dt)
            })

            //console.log(ray)
        })
        

        
        return scene;
    };
}


class Wheel {
    name: string
    mesh: Mesh
    /** Relative to spaceship position */
    position: Vector3
    radius = 0.5
    springLength = 2
    springRestLength = 1
    springMaxLength = 10
    springStiffness = 10
    dampingCompression = 1.4
    dampingRelaxation = 2.3
    maxSuspensionForce = 100000
    maxSuspensionTravel = 0.3

    

    scene: Scene

    spaceshipBody: PhysicsBody

    constructor(name: string, position: Vector3,  scene: Scene, spaceshipMesh: Mesh, spaceshipBody: PhysicsBody) { 
        this.name = name
        this.mesh = new Mesh(name, scene)
        this.mesh.parent = spaceshipMesh
        this.mesh.position = position
        this.position = position
        this.scene = scene
        this.spaceshipBody = spaceshipBody
    }

    onUpdate(dt: number) {
        this.raycast(dt)
    }

    raycast(dt: number) {
        const wheelDown = new Vector3(0, -1, 0);
        const worldDirection = Vector3.TransformNormal(wheelDown, this.getWorldMatrix());
        const rayLength = this.springRestLength + this.radius;
        const raycastResult = this.scene.getPhysicsEngine()?.raycast(this.getAbsolutePosition(), this.getAbsolutePosition().clone().add(worldDirection.scaleInPlace(rayLength)), {collideWith: 2})
    
        if(raycastResult) {
            // console.log('raycastresult', raycastResult)
            if(raycastResult.body !== this.spaceshipBody) {
                const distance = Vector3.Distance(this.getAbsolutePosition(), raycastResult.hitPointWorld);
                const offset = this.springRestLength - distance
                const force = offset * this.springStiffness * this.spaceshipBody.getMassProperties().mass!

                // damping
                const project = Vector3.Dot(raycastResult.hitNormalWorld, new Vector3())
                const spaceshipVelocityAtContactPoint = new Vector3()
                this.velocityAt(this.spaceshipBody, raycastResult.hitPointWorld, spaceshipVelocityAtContactPoint)


                const impulse = new Vector3().copyFrom(raycastResult.hitNormalWorld).scaleInPlace(force * dt);
                this.spaceshipBody.applyImpulse(impulse, raycastResult.hitPointWorld)
            }
        }
    }

    getAbsolutePosition (){
        return this.mesh.getAbsolutePosition()
    }

    getWorldMatrix() {
        return this.mesh.getWorldMatrix()
    }

    
    velocityAt (body: PhysicsBody, pos: Vector3, res: Vector3) {
        const tmpVel2 = new Vector3()
        body.getObjectCenterWorldToRef(tmpVel2)
        pos.subtractToRef(tmpVel2, res)
        body.getAngularVelocityToRef(tmpVel2)
        Vector3.CrossToRef(tmpVel2, res, res)
        body.getLinearVelocityToRef(tmpVel2)
        res.addInPlace(tmpVel2)
        return res;
    }
}

export default new PhysicsSceneWithHavok();
