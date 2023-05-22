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
    # Spline joint
    mechanism = Mechanism{Float64}("VirtualMechanism")
    cart_frame = add_frame!(mechanism, "cart_frame")
    TR = Transform(initial_ee_pos, zero(Rotor))
    rail_joint = Rail(build_spline(), TR, 1)
    add_joint!(mechanism, rail_joint, root_frame(mechanism), cart_frame)
    # Inertia
    push!(mechanism, PointMass(1.0, cart_frame))

    mechanism
end

function combine_mechanisms!(mechanism, virtual_mechanism)
    # Spline joint
    new_cart_frame_idx = add_frame!(mechanism, "cart_frame")
    rail_joint = joints(virtual_mechanism)[end]
    new_joint = Rail(
        rail_joint.spline,
        rail_joint.transform,
        Int(ndof(mechanism) + 1)
    )
    add_joint!(mechanism, new_joint , root_frame(mechanism), new_cart_frame_idx)
    
    # Spring/Damper
    stiffness = @SMatrix([100.0 0.0   0.0;
                        0.0   100.0 0.0;
                        0.0   0.0   100.0])
    damping = @SMatrix([5.0  0.0  0.0;
                        0.0  5.0  0.0;
                        0.0  0.0  5.0])
    EE_frame = find_frame_index(mechanism, "EE_frame")
    push!(mechanism, LinearSpring(stiffness, new_cart_frame_idx, EE_frame))
    push!(mechanism, LinearDamper(damping, root_frame(mechanism), new_cart_frame_idx))
    mechanism
end

# function spring_forces(m, vm, q_robot, q_cart, EE_frame, cart_frame, stiffness)
#     hess_robot = RobotDynamics.Hessians.hessian(q -> kinematics(m, q), q_robot)
#     hess_cart = RobotDynamics.Hessians.hessian(q -> kinematics(vm, q), q_cart)
#     tf_parent = RobotDynamics.value(hess_cart)[cart_frame]
#     tf_child = RobotDynamics.value(hess_robot)[EE_frame]
#     J_parent = RobotDynamics.jacobian(hess_cart)[cart_frame]
#     J_child = RobotDynamics.jacobian(hess_robot)[EE_frame]
#     u_cart, u_robot = RobotDynamics.linear_spring_generalized_forces(
#         stiffness, tf_parent, tf_child, J_parent, J_child)
#     u_cart, u_robot
# end

function spline_demo()
    # Load mechanism
    mechanism = parseRSON("../RSONs/rsons/franka_panda/panda_kinematics.rson")    
    # mechanism = parseRSON("/home/daniellarby/RSONs/rsons/franka_panda/panda_kinematics.rson")    
    EE_frame = find_frame_index(mechanism, "EE_frame")
    # m = compile(mechanism)

    # Get starting position for spline
    # q_robot_init = @SVector(ones(7))
    pipes, q_robot_init = open_communication(@SVector([i for i in 1:7]))
    ref = origin(kinematics(compile(mechanism), q_robot_init)[EE_frame])
    
    # Append virtual mechanism
    virtual_mechanism = build_virtual_mechanism(ref)
    mechanism = combine_mechanisms!(mechanism, virtual_mechanism)

    cart_frame = find_frame_index(mechanism, "cart_frame")
    m = compile(mechanism)
    vm = compile(virtual_mechanism)

    # Initial state
    q_robot, q̇_robot = q_robot_init, @SVector(zeros(7))
    q_cart, q̇_cart = SVector(0.0), SVector(0.0)
    u_cart = SVector(0.0)

    # Safety check...
    begin
        q_init = vcat(q_robot_init, q_cart)
        ee_init = origin(kinematics(m, q_init)[EE_frame])
        cart_init = origin(kinematics(m, q_init)[cart_frame])
        # @show ref, ee_init, cart_init
        @assert all(ee_init - cart_init  .== 0.0)
    end

    qʳ_idxs = @SVector([i for i in 1:7])
    qᶜ_idxs = @SVector([i for i in 8:8])
    q̇ʳ_idxs = @SVector([i for i in 9:15])
    q̇ᶜ_idxs = @SVector([i for i in 16:16])

    function loop_func(i, q_robot, q̇_robot, q_cart, q̇_cart, u_cart)
        # SIMULATE CART
        q̈_cart = dynamics(vm, q_cart, q̇_cart, u_cart)
        q_cart += 1e-3 * q̇_cart
        q̇_cart += 1e-3 * q̈_cart

        # Create full state vector
        q = vcat(q_robot, q_cart)
        q̇ = vcat(q̇_robot, q̇_cart)

        # Compute and write torques
        u = generalized_force(m, q, q̇)
        u_robot, u_cart = u[qʳ_idxs], u[qᶜ_idxs]
        write_robot_torques(pipes, u_robot)

        # Read robot state
        x = parse_robot_state(pipes; timeout=0.2)
        q_robot, q̇_robot = x[qʳ_idxs], x[q̇ʳ_idxs]

        q_robot, q̇_robot, q_cart, q̇_cart, u_cart
    end
    
    args = (q_robot, q̇_robot, q_cart, q̇_cart, u_cart)
    warmup(loop_func, args...)
    println("Ready...")
    mainloop(loop_func, args...)
end

spline_demo()