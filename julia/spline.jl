include("./control.jl")


function build_spline(;w = 0.5, x0=0.2, L=1.0, h=0.9, Nk = 3, Nr = 4)
    forward = Vector(LinRange(x0, x0+L, Nk))
    backward = Vector(reverse(forward))
    spline = Matrix{Float64}(undef, 0, 3);

    for i = 0:(Nr-1)
        knots = (i%2)!=1 ? forward : backward 
        spline = vcat(
            spline, 
            hcat(knots, w*i*ones(size(knots)), h*ones(size(knots)))
        )
    end
    CubicSpline(spline)
end

function build_virtual_mechanism(initial_ee_pos)
    vm = Mechanism{Float64}("CartVirtualMechanism")
    cart_frame = add_frame!(vm, "cart_frame")
    TR = Transform(initial_ee_pos, zero(Rotor))
    rail_joint = Rail(build_spline(), TR, Int(1))
    add_joint!(mechanism, rail_joint, root_frame(mechanism), cart_frame)
    vm, cart_frame
end

function spline_demo()
    # Load mechanism
    mechanism = parseRSON("/home/daniellarby/RSONs/rsons/franka_panda/pandaDummyInstrument.rson")
    EE_frame = find_frame_index(mechanism, "EE_frame")
    q_idxs = @SVector([i for i in 1:7])
    q̇_idxs = @SVector([i for i in 8:14])
    m = compile(mechanism)

    pipes, q_init = open_communication(q_idxs)

    # Build virtual mechanism
    ref = origin(kinematics(compile(mechanism), q_init)[EE_frame])
    virtual_mechanism = build_virtual_mechanism(ref)
    vm, cart_frame = compile(virtual_mechanism)

    # Define spring
    stiffness = @SMatrix([100.0 0.0   0.0;
                          0.0   100.0 0.0;
                          0.0   0.0   100.0])


    q_cart, q̇_cart = SVector(0.0), SVector(0.0)

    function loop_func(i, u_robot, u_cart)
        # start = time()
        # SIMULATE CART
        q̈_cart = dynamics(vm, q_cart, q̇_cart, u_cart)
        q_cart, q̇_cart += 1e-3 .* (q̇_cart, q̈_cart) 

        write_robot_torques(pipes, u_robot)
        x = parse_robot_state(pipes; timeout=0.2)
        q_robot, q̇_robot = x[q_idxs], x[q̇_idxs]

        hess_robot = RobotDynamics.Hessians.hessian(q -> kinematics(m, q), q_robot)
        hess_cart = RobotDynamics.Hessians.hessian(q -> kinematics(vm, q), q_robot)
        
        tf_parent = value(hess_cart)[cart_frame]
        tf_child = value(hess_robot)[EE_frame]
        J_parent = jacobian(hess_cart)[cart_frame]
        J_child = jacobian(hess_robot)[EE_frame]
        
        u_robot, u_cart = RobotDynamics.linear_spring_generalized_forces(
            stiffness, tf_parent, tf_child, J_parent, J_child)
        
        u_robot, u_cart
    end
    
    zero_torque = 0.0*q_init
    warmup(loop_func, zero_torque)
    println("Ready...")
    mainloop(loop_func, zero_torque)
end

spline_demo()