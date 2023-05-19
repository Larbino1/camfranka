include("./control.jl")


function build_spline(;w = 0.5, x0=0.0, L=1.0, h=0.0, Nk = 3, Nr = 4)
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
    RobotDynamics.CubicSpline(spline)
end

function build_virtual_mechanism(initial_ee_pos)
    vm = Mechanism{Float64}("CartVirtualMechanism")
    cart_frame = add_frame!(vm, "cart_frame")
    TR = Transform(initial_ee_pos, zero(Rotor))
    rail_joint = Rail(build_spline(), TR, Int(1))
    add_joint!(vm, rail_joint, root_frame(vm), cart_frame)
    push!(vm, PointMass(1.0, cart_frame))
    damping = @SMatrix([5.0  0.0  0.0;
                        0.0  5.0  0.0;
                        0.0  0.0  5.0])
    push!(vm, LinearDamper(damping, root_frame(vm), cart_frame))
    vm, cart_frame
end

function spring_forces(m, vm, q_robot, q_cart, EE_frame, cart_frame, stiffness)
    hess_robot = RobotDynamics.Hessians.hessian(q -> kinematics(m, q), q_robot)
    hess_cart = RobotDynamics.Hessians.hessian(q -> kinematics(vm, q), q_cart)
    tf_parent = RobotDynamics.value(hess_cart)[cart_frame]
    tf_child = RobotDynamics.value(hess_robot)[EE_frame]
    J_parent = RobotDynamics.jacobian(hess_cart)[cart_frame]
    J_child = RobotDynamics.jacobian(hess_robot)[EE_frame]
    u_cart, u_robot = RobotDynamics.linear_spring_generalized_forces(
        stiffness, tf_parent, tf_child, J_parent, J_child)
    u_cart, u_robot
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
    virtual_mechanism, cart_frame = build_virtual_mechanism(ref)
    vm = compile(virtual_mechanism)
    q_cart, q̇_cart = SVector(0.0), SVector(0.0)

    @assert all(origin(kinematics(m, q_init)[EE_frame]) - origin(kinematics(vm, q_cart)[cart_frame]) .== 0.0)

    # Define spring
    stiffness = @SMatrix([100.0 0.0   0.0;
                          0.0   100.0 0.0;
                          0.0   0.0   100.0])



    function loop_func(i, u_cart, u_robot)
        # start = time()
        # SIMULATE CART
        
        q̈_cart = dynamics(vm, q_cart, q̇_cart, u_cart)

        # q̈_cart = SVector(0.0) # for testing
        q_cart += 1e-3 * q̇_cart
        q̇_cart += 1e-3 * q̈_cart

        write_robot_torques(pipes, u_robot)
        x = parse_robot_state(pipes; timeout=0.2)
        q_robot, q̇_robot = x[q_idxs], x[q̇_idxs]        
        
        u_cart, u_robot = spring_forces(m, vm, q_robot, q_cart, EE_frame, cart_frame, stiffness)
        u_robot = -u_robot
        u_cart, u_robot
    end
    
    zero_torque = (0.0*q_cart, 0.0*q_init)
    warmup(loop_func, zero_torque...)
    println("Ready...")
    mainloop(loop_func, zero_torque...)
end

spline_demo()