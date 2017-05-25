using PyPlot, DifferentialEquations

## Define the variables
ζ = 1.0
ubar = 0.1*ζ
tf = 40.0
t0 = 0.0
Θ0 = π + 0.1*rand(1) - 0.05
Θ0 = Θ0[1]
Θdot0 = 0.0
Href = 1.0;
B = [0.; 1.]

## Integrate with Runge-Kutta
# Drift Vector Field
function f(x)
    dx = [x[2];
          ζ*sin(x[1])]
    return dx
end

# Controller from trajectory optimization
function k(t0,t,x)
  H = 1/2*x[2]^2 + cos(x[1]);
  Htilde = H - Href
  if norm(x) > 0.1
    uu = -5*x[2]*Htilde
  else
    uu = -10.0*sin(x[1])-7.0*x[2]
    # uu = -1/R*B'*S*x
    uu = uu[1]
  end
  if uu >= ubar
    uu = ubar
  end
  if uu <= -ubar
    uu = -ubar
  end
  return uu
end

# Closed-Loop Vector Field
function clvf(t,x)
  return f(x) + B*k(0, t, x)
end


# Solve using Runge-Kutta
torque = Array{Float64}(0,1)
x0 = [Θ0, Θdot0]
tspan = (t0,tf)
# prob = ODEProblem(clf(τ), x0, tspan)
prob = ODEProblem(clvf, x0, tspan)
sol = DifferentialEquations.solve(prob, RK4(), dt = 0.01)
x = sol[:,:]'
tt = sol.t
for i in 1:length(tt)
  torque = [torque; k(t0,tt[i],x[i,:])]
end


## Plotting

fig=figure(1)
clf()
subplot(2,2,1)
plot(tt, x[:,1], linewidth=2.)
xlim(t0, tf)
# ylim(-3.7, 0.02)
xlabel(L"t \; [sec]", fontsize=15)
ylabel(L"$\theta \; [rad]$", fontsize=15)
# axis("tight")
subplot(2,2,2)
plot(tt, x[:,2], linewidth=2.)
xlabel(L"t \; [sec]", fontsize=15)
ylabel(L"\dot{\theta} \; [\frac{rad}{s}]", fontsize=15)
xlim(t0, tf)
# ylim(-0.78, 1.2)
subplot(2,2,(3,4))
plot(tt, torque, linewidth=2.)
xlabel(L"t \; [sec]", fontsize=15)
ylabel(L"\tau \; [Nm]", fontsize=15)
xlim(t0, tf)
# ylim(-0.77, 0.77)
tight_layout()

savefig("pendulum_energy_law.eps", format="eps")

## Visualize the output

using DrakeVisualizer
using CoordinateTransformations
import GeometryTypes: HyperRectangle, Vec, HomogenousMesh
import ColorTypes: RGBA

# Launch the viewer application if it isn't running already:
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window()

# First, we'll create a simple geometric object
box = HyperRectangle(Vec(0.,0,0), Vec(1.,1,-5))
blue_box = GeometryData(box, RGBA(0., 0., 1, 0.5))

# First we make an empty visualizer
vis = Visualizer()

# We can access a particular path within the visualizer with indexing notation:
vis[:group1]

# We load geometries using the same path notation:
blue_box_vis = setgeometry!(vis[:group1][:bluebox], blue_box)

for i in 1:length(tt)
  settransform!(blue_box_vis, LinearMap(AngleAxis(x[i,1]-pi,0,1,0)))
  sleep(0.01)
end
