import {Engine} from "@babylonjs/core/Engines/engine";
import {Scene} from "@babylonjs/core/scene";
import {ArcRotateCamera} from "@babylonjs/core/Cameras/arcRotateCamera";
import {Quaternion, Vector3} from "@babylonjs/core/Maths/math.vector";
import {HemisphericLight} from "@babylonjs/core/Lights/hemisphericLight";
import {CreateGround} from "@babylonjs/core/Meshes/Builders/groundBuilder";
import "@babylonjs/core/Physics/physicsEngineComponent";

// If you don't need the standard material you will still need to import it since the scene requires it.
import "@babylonjs/core/Materials/standardMaterial";
import {CreateSceneClass} from "../createScene";
import {havokModule} from "../externals/havok";
import {PhysicsShapeBox} from "@babylonjs/core/Physics/v2/physicsShape";
import {PhysicsBody} from "@babylonjs/core/Physics/v2/physicsBody";
import {PhysicsMotionType} from "@babylonjs/core/Physics/v2/IPhysicsEnginePlugin";
import {HavokPlugin} from "@babylonjs/core/Physics/v2/Plugins/havokPlugin";
import {
    Color3,
    CreateBox,
    GizmoManager,
    Mesh,
    MeshBuilder,
    PhysicsViewer,
    Ray,
    RayHelper,
    StandardMaterial,
    KeyboardEventTypes, Gizmo, AxesViewer, Tools
} from "@babylonjs/core";


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

        scene.enablePhysics(new Vector3(0, -9.81, 0), new HavokPlugin(true, await havokModule));


        //  Configure the spaceship body
        const spaceshipShape = new PhysicsShapeBox(new Vector3(0, 0, 0),
            Quaternion.Identity(),
            new Vector3(2, 1, 4),
            scene
        );
        const spaceshipMesh = CreateBox("spaceship", {width: 2, height: 1, depth: 4}, scene);
        spaceshipMesh.position.y = 4;
        const spaceshipBody = new PhysicsBody(spaceshipMesh, PhysicsMotionType.DYNAMIC, false, scene);
        spaceshipShape.material = {friction: 0.2, restitution: 0.6};
        spaceshipBody.shape = spaceshipShape;
        spaceshipBody.setMassProperties({
            mass: 800,
            inertia: new Vector3(1, 1, 1),
            centerOfMass: new Vector3(0, -0.9, 0)
        });


        //Configure the ground
        const ground = CreateGround("ground", {width: 1000, height: 1000}, scene);
        const groundMaterial = new StandardMaterial("matBox", scene);
        groundMaterial.diffuseColor = new Color3(0.4, 0.1, 0.1);
        ground.material = groundMaterial
        const groundShape = new PhysicsShapeBox(new Vector3(0, 0, 0), Quaternion.Identity(), new Vector3(10000, 0.1, 1000), scene);
        const groundBody = new PhysicsBody(ground, PhysicsMotionType.STATIC, false, scene);
        groundShape.material = {friction: 0.2, restitution: 0.8};
        groundBody.shape = groundShape;
        groundBody.setMassProperties({mass: 0});
        groundShape.filterMembershipMask = 2;


        // Create Wheels
        const wheels = [
            new FNWheel("lf", new Vector3(-0.95, 0, 1.8), scene, spaceshipMesh, spaceshipBody),
            new FNWheel("rf", new Vector3(0.95, 0, 1.8), scene, spaceshipMesh, spaceshipBody),
            new FNWheel("lb", new Vector3(-0.95, 0, -1.8), scene, spaceshipMesh, spaceshipBody),
            new FNWheel("rb", new Vector3(0.95, 0, -1.8), scene, spaceshipMesh, spaceshipBody),
        ]

        // Input handle
        const input = FNInput.getInstance(scene)

        scene.registerBeforeRender(() => {
            const dt = engine.getDeltaTime() / 1000;
            wheels.forEach((w) => {

                w.onUpdate(dt)
            })

            if (input.controls.jump) {
                const upVector = Vector3.Up().scale(15000 * scene.getEngine().getDeltaTime() / 1000)
                spaceshipBody.applyImpulse(upVector, spaceshipMesh.position)
            }
        })

        return scene;
    };
}


class FNWheel {
    name: string
    mesh: Mesh
    /** Relative to spaceship position */
    position: Vector3
    radius = 0.5
    springRestLength = 1
    springStiffness = 10
    dampingCompression = 1.4
    dampingRelaxation = 2.3
    maxSuspensionForce = 100000
    maxSuspensionTravel = 0.3

    suspensionRelativeVelocity = 0
    invContactDotSuspension = 1


    scene: Scene

    spaceshipBody: PhysicsBody

    steering: FNAckSteering

    constructor(name: string, position: Vector3, scene: Scene, spaceshipMesh: Mesh, spaceshipBody: PhysicsBody) {
        this.name = name
        this.mesh = new Mesh(name, scene)
        this.mesh.parent = spaceshipMesh
        this.mesh.position = position
        this.position = position
        this.scene = scene
        this.spaceshipBody = spaceshipBody

        this.steering = new FNAckSteering(scene, this)

        FNDebug.AxisView(scene, this.mesh)
    }

    onUpdate(dt: number) {
        this.raycast(dt)
    }

    raycast(dt: number) {
        const wheelDown = new Vector3(0, -1, 0);
        const worldDirection = Vector3.TransformNormal(wheelDown, this.getWorldMatrix()).normalizeToNew();
        const rayLength = this.springRestLength + this.radius;
        const raycastResult = this.scene.getPhysicsEngine()?.raycast(this.getAbsolutePosition(), this.getAbsolutePosition().clone().add(worldDirection.clone().scaleInPlace(rayLength)), {collideWith: 2})

        if (raycastResult) {
            if (raycastResult.body !== this.spaceshipBody) {
                const distance = Vector3.Distance(this.getAbsolutePosition(), raycastResult.hitPointWorld) - this.radius;
                const offset = this.springRestLength - distance

                // TODO check is in contact
                // Checking whether the wheel is traveling exactly the opposite direction from the normal or not
                // -1 = opposite direction, 1 = same direction
                const project = Vector3.Dot(raycastResult.hitNormalWorld, worldDirection)

                const spaceshipVelocityAtContactPoint = new Vector3()
                this.velocityAt(this.spaceshipBody, raycastResult.hitPointWorld, spaceshipVelocityAtContactPoint)

                const projectedVelocity = Vector3.Dot(spaceshipVelocityAtContactPoint, raycastResult.hitNormalWorld);


                // Going in the same direction as the normal
                if (project >= -0.1) {
                    this.suspensionRelativeVelocity = 0
                    this.invContactDotSuspension = 1 / 0.1
                } else {
                    // Going against the normal
                    const inv = -1 / project
                    this.suspensionRelativeVelocity = projectedVelocity * inv
                    this.invContactDotSuspension = inv
                }

                let force = offset * this.springStiffness * this.invContactDotSuspension

                // Damper
                let suspensionDamping = 0


                if (this.suspensionRelativeVelocity < 0) {
                    suspensionDamping = this.dampingCompression
                } else {
                    suspensionDamping = this.dampingRelaxation
                }

                force -= suspensionDamping * this.suspensionRelativeVelocity
                const suspensionForce = force * this.spaceshipBody.getMassProperties().mass!
                if (suspensionForce > 0) {
                    const impulse = new Vector3().copyFrom(raycastResult.hitNormalWorld).scaleInPlace(suspensionForce * dt);
                    this.spaceshipBody.applyImpulse(impulse, raycastResult.hitPointWorld)
                }


            }
        } else {
            // Get suspension back at rest
            this.suspensionRelativeVelocity = 0
            this.invContactDotSuspension = 1.0
        }

        // Steering applied to front tires
        if (this.name == 'lf' || this.name == 'rf') {
            const ackAngle = this.steering.update(dt)
            const entityRot = this.mesh.rotationQuaternion?.clone() ?? new Quaternion();
            entityRot.copyFrom(Quaternion.RotationYawPitchRoll(
                Tools.ToRadians(ackAngle * 3),
                entityRot.toEulerAngles().x,
                entityRot.toEulerAngles().z
            ))
            this.mesh.rotationQuaternion = entityRot;
        }

    }

    getAbsolutePosition() {
        return this.mesh.getAbsolutePosition()
    }

    getWorldMatrix() {
        return this.mesh.getWorldMatrix()
    }

    velocityAt(body: PhysicsBody, pos: Vector3, res: Vector3) {
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


// Ackermann
class FNAckSteering {
    steeringRate: number = 10;
    turnRadius: number = 10;
    wheelBase: number = 2.5;
    rearTrack: number = 1.5;

    wheel: FNWheel
    input: FNInput

    private ackermannAngleLeft = 0;
    private ackermannAngleRight = 0;
    private steerInput = 0;


    constructor(scene: Scene, wheel: FNWheel) {
        this.wheel = wheel;
        this.input = FNInput.getInstance(scene)
    }

    private calculateSteerInput(dt: number) {
        const {left, right} = this.input.controls

        if (left && !right) {
            this.steerInput = FNHelper.clamp(this.steerInput - this.steeringRate * dt, -1, 1);
        } else if (right && !left) {
            this.steerInput = FNHelper.clamp(this.steerInput + this.steeringRate * dt, -1, 1);
        } else {
            // When neither "A" nor "D" is pressed, steer back to zero
            if (this.steerInput < 0) {
                this.steerInput = FNHelper.clamp(this.steerInput + (this.steeringRate + 1) * dt, -1, 0);
            } else if (this.steerInput > 0) {
                this.steerInput = FNHelper.clamp(this.steerInput - (this.steeringRate + 1) * dt, 0, 1);
            }
        }

    }

    private calculateAckAngles() {
        if (this.steerInput > 0) {

            this.ackermannAngleLeft = 180 / Math.PI * Math.atan(this.wheelBase / (this.turnRadius + (this.rearTrack / 2))) * this.steerInput;
            this.ackermannAngleRight = 180 / Math.PI * Math.atan(this.wheelBase / (this.turnRadius - (this.rearTrack / 2))) * this.steerInput;
        } else if (this.steerInput < 0) {
            this.ackermannAngleLeft = 180 / Math.PI * Math.atan(this.wheelBase / (this.turnRadius - (this.rearTrack / 2))) * this.steerInput;
            this.ackermannAngleRight = 180 / Math.PI * Math.atan(this.wheelBase / (this.turnRadius + (this.rearTrack / 2))) * this.steerInput;
        } else {
            this.ackermannAngleLeft = 0;
            this.ackermannAngleRight = 0;
        }
    }

    // Should be called each frame
    update(dt: number) {
        this.calculateSteerInput(dt)
        this.calculateAckAngles()
        return this.wheel.name === 'lf' ? this.ackermannAngleLeft : this.ackermannAngleRight
    }

}


class FNInput {
    controls: {
        forward: boolean,
        backward: boolean,
        left: boolean,
        right: boolean,
        brake: boolean,

        jump: boolean
    }

    private static instance: FNInput | null = null;

    private constructor(scene: Scene) {
        this.controls = {
            forward: false,
            backward: false,
            left: false,
            right: false,
            brake: false,
            jump: false
        }
        scene.onKeyboardObservable.add((k) => {
            switch (k.type) {
                case KeyboardEventTypes.KEYDOWN:
                    if (k.event.key == 'w') this.controls.forward = true;
                    if (k.event.key == 's') this.controls.backward = true;
                    if (k.event.key == 'a') this.controls.left = true;
                    if (k.event.key == 'd') this.controls.right = true;
                    if (k.event.key == ' ') this.controls.brake = true;

                    if (k.event.key == 'f') this.controls.jump = true;
                    break;
                case KeyboardEventTypes.KEYUP:
                    if (k.event.key == 'w') this.controls.forward = false;
                    if (k.event.key == 's') this.controls.backward = false;
                    if (k.event.key == 'a') this.controls.left = false;
                    if (k.event.key == 'd') this.controls.right = false;
                    if (k.event.key == ' ') this.controls.brake = false;
                    if (k.event.key == 'f') this.controls.jump = false;
                    break;
            }
        });
    }

    public static getInstance(scene: Scene): FNInput {
        if (!FNInput.instance) {
            FNInput.instance = new FNInput(scene);
        }
        return FNInput.instance;
    }
}

class FNDebug {

    static DrawLine(scene: Scene, start: Vector3, direction: Vector3, length: number, color: Color3) {
        const end = start.add(direction.scale(length));
        const points = [start, end];
        const lines = MeshBuilder.CreateLines("line", {points}, scene);
        lines.color = color || new Color3(0, 0, 0);
        return lines
    }

    static AxisView(scene: Scene, mesh: Mesh) {
        const aw = new AxesViewer(scene)
        aw.xAxis.parent = mesh;
        aw.yAxis.parent = mesh;
        aw.zAxis.parent = mesh;
    }
}

class FNHelper {
    static clamp(value: number, min: number, max: number) {
        return Math.min(Math.max(value, min), max);
    }
}

export default new PhysicsSceneWithHavok();
