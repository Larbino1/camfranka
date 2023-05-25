include("./control.jl")


function build_zigzag_spline(; w=0.0025, x0=0.0, L=0.05, h=0.0, Nk=5, Nr=25)
    forward = Vector(LinRange(x0, x0 + L, Nk))
    backward = Vector(reverse(forward))
    spline = Matrix{Float64}(undef, 0, 3)

    for i = 0:(Nr-1)
        knots = (i % 2) != 1 ? forward : backward
        spline = vcat(
            spline,
            hcat(knots, w * i * ones(size(knots)), h * ones(size(knots)))
        )
    end
    RobotDynamics.CubicSpline(spline)
end

function add_cart!(mechanism, initial_ee_transform)
    # Spline joint
    cart_frame = add_frame!(mechanism, "cart_frame")
    TR = Transform(origin(initial_ee_transform), zero(Rotor))
    RailIdx = ndof(mechanism) + 1
    rail_joint = Rail(build_zigzag_spline(), TR, RailIdx)
    add_joint!(mechanism, rail_joint, root_frame(mechanism), cart_frame)
    # Joint Inertia/damping
    push!(mechanism, JointInerter(0.2, RailIdx))
    push!(mechanism, JointDamper(20.0, RailIdx))

    # EE Spring/Damper
    stiffness = @SMatrix([2000.0 0.0    0.0
                          0.0    2000.0 0.0
                          0.0    0.0    2000.0])
    damping = @SMatrix([5.0 0.0 0.0
                        0.0 5.0 0.0
                        0.0 0.0 5.0])
    EE_frame = find_frame_index(mechanism, "burr_frame")
    push!(mechanism, LinearSpring(stiffness, cart_frame, EE_frame))
    push!(mechanism, LinearDamper(damping, root_frame(mechanism), cart_frame))
    mechanism
end

function add_prismatic_extension!(mechanism, initial_ee_transform)
    ####################
    # Add prismatic extension
    ####################
    EE_frame = find_frame_index(mechanism, "burr_frame") # TODO this should always agree with initial_EE_transform
    rcm_offset = SVector(-0.2, 0.0, 0.0) # RCM offset from burr in EE frame
    rcm = initial_ee_transform * rcm_offset
    slider_frame = add_frame!(mechanism, "slider")
    PrismaticIdx = ndof(mechanism) + 1
    TP = Transform(rcm_offset) # This ensures that starting at q=0 starts the slider at the rcm
    JP = Prismatic(SVector(1.0, 0.0, 0.0), TP, PrismaticIdx)
    add_joint!(mechanism, JP, EE_frame, slider_frame)
    push!(mechanism, JointInerter(1.0, PrismaticIdx))
    push!(mechanism, JointDamper(10.0, PrismaticIdx))
    rcm_frame = add_frame!(mechanism, "rcm_frame")
    add_joint!(mechanism, Rigid(Transform(rcm)), root_frame(mechanism), rcm_frame)
    push!(mechanism, LinearSpring(1000.0 * identity(3), rcm_frame, slider_frame))
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
    # Load mechanism and remove inertial components so that they do not contribute
    # to computed generalized_force when we have constructed the control mechanism
    mechanism = parseRSON("../../RSONs/rsons/franka_panda/pandaDremelMount.rson")
    empty!(inertances(mechanism))
    empty!(generic_components(mechanism))

    EE_frame = find_frame_index(mechanism, "burr_frame")

    # Get starting position for spline
    pipes, q_robot_init = open_communication(@SVector([i for i in 1:7]))
    initial_EE_transform = kinematics(compile(mechanism), q_robot_init)[EE_frame]

    add_cart!(mechanism, initial_EE_transform)
    add_prismatic_extension!(mechanism, initial_EE_transform)
    
    # Get useful things
    L = RobotDynamics.Splines.integrate_length(joints(mechanism)[end-2].spline) # TODO remove magic number
    cart_frame = find_frame_index(mechanism, "cart_frame")
    slider_frame = find_frame_index(mechanism, "slider")
    rcm_frame = find_frame_index(mechanism, "rcm_frame")

    # Compile mechanism
    m = compile(mechanism)

    # Initial state
    q_robot, q̇_robot = q_robot_init, @SVector(zeros(7))
    q_virtual, q̇_virtual = SVector(0.0, 0.0), SVector(0.0, 0.0)
    u_virtual = SVector(0.0, 0.0)

    # Safety check...
    begin
        q_init = vcat(q_robot, q_virtual)
        q̇_init = vcat(q̇_robot, q̇_virtual)
        initial_transforms = kinematics(m, q_init)
        ee_init = origin(initial_transforms[EE_frame])
        cart_init = origin(initial_transforms[cart_frame])
        slider_init = origin(initial_transforms[slider_frame])
        rcm_init = origin(initial_transforms[rcm_frame])
        # @show ref, ee_init, cart_init
        @assert all(ee_init - cart_init .≈ 0.0)
        @show slider_init - rcm_init
        @assert all(slider_init - rcm_init .≈ 0.0)
        @show generalized_force(m, q_init, q̇_init)
        @assert all(generalized_force(m, q_init, q̇_init) .≈ 0.0)
        # fig = Makie.Figure()
        # ax = Makie.Axis3(fig[1,1]; aspect=:data)
        # robotplot!(ax, m, q_init, visuals(mechanism))
        # sp = RobotDynamics.Splines.splineplot!(ax, joints(virtual_mechanism)[1].spline)
        # translate!(sp, cart_init)
        # display(fig)
    end

    uʳ_idxs = @SVector([i for i in 1:7])
    uᶜ_idxs = @SVector([i for i in 8:9])

    function loop_func(i, q_robot, q̇_robot, q_virtual, q̇_virtual)
        # q̈_virtual = dynamics(vm, q_virtual, q̇_virtual, u_virtual)
        # q_virtual += 1e-3 * q̇_virtual
        # q̇_virtual += 1e-3 * q̈_virtual

        # Create full state vector
        q = vcat(q_robot, q_virtual)
        q̇ = vcat(q̇_robot, q̇_virtual)

        # Compute forces
        u = generalized_force(m, q, q̇)
        u_robot, u_virtual = u[uʳ_idxs], u[uᶜ_idxs]
        # Apply driving force if still on spline
        u_virtual = q_virtual[1] < L ? u_virtual .+ SVector(5.0, 0.0) : u_virtual

        # SIMULATE virtual mechanism
        M = inertance_matrix(m, q, q̇)
        M_virtual = M[uᶜ_idxs, uᶜ_idxs]
        q̈_virtual = M_virtual \ u_virtual
        q_virtual += 1e-3 * q̇_virtual
        q̇_virtual += 1e-3 * q̈_virtual

        # Send forces to actual robot
        write_robot_torques(pipes, u_robot)

        if ((i % 100) == 0)
            println(q_virtual)
        end

        # Read robot state
        q_robot, q̇_robot = parse_robot_pos_vel(pipes; timeout=0.2)
        q_robot, q̇_robot, q_virtual, q̇_virtual
    end

    args = (q_robot, q̇_robot, q_virtual, q̇_virtual)
    warmup(loop_func, args...)
    println("Ready...")
    mainloop(loop_func, args...)
end

spline_demo()