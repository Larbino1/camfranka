include("./control.jl")

function spring_demo()
    # Load mechanism
    mechanism = RobotDynamics.RSON.parseRSON("/home/daniellarby/RSONs/rsons/franka_panda/pandaDummyInstrument.rson")
    EE_frame = find_frame_index(mechanism, "EE_frame")
    q_idxs = @SVector([i for i in 1:7])
    q̇_idxs = @SVector([i for i in 8:14])

    pipes, q_init = open_communication(q_idxs)

    # Set spring endpoint
    ref = origin(kinematics(compile(mechanism), q_init)[EE_frame])
    ref_frame = add_frame!(mechanism, "ref_frame")
    ref_joint = Rigid(Transform(ref, zero(Rotor)))
    add_joint!(mechanism, ref_joint, root_frame(mechanism), ref_frame)
    # Add spring
    stiffness = @SMatrix([100.0 0.0   0.0;
                          0.0   100.0 0.0;
                          0.0   0.0   100.0])
    spring = LinearSpring(stiffness, ref_frame, EE_frame)
    m = compile(mechanism)
    
    function plot_func(q)
        # println("Plotting...")
        # q_plot[] = q
        println(q)
        # sleep(.00001)
    end

    function loop_func(i, control_torque)
        # start = time()
        write_robot_torques(pipes, control_torque)
        x = parse_robot_state(pipes; timeout=0.2)
        q, q̇ = x[q_idxs], x[q̇_idxs]
        
        hessianresult = RobotDynamics.Hessians.hessian(q -> kinematics(m, q), q)
        control_torque = generalized_force(spring, hessianresult, q̇)
        if ( (i % 100) == 0)
            plot_func(q)
            # println(control_torque)
        end
        
        (control_torque,)
    end
    
    zero_torque = 0.0*q_init
    warmup(loop_func, zero_torque)
    println("Ready...")
    mainloop(loop_func, zero_torque)
end

spring_demo()