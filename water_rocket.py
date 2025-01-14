import pymunk
import pymunk.pygame_util
import pygame
import tkinter as tk
from tkinter import ttk
import math
import time
from threading import Thread

# Initialize Pygame and PyMunk
pygame.init()
width, height = 600, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
draw_options = pymunk.pygame_util.DrawOptions(screen)

# PyMunk space setup
space = pymunk.Space()
space.gravity = (0, 980)  # Gravity pointing downward (in pymunk coordinate system)

# Constants/Variables
discharge_coefficient = 0.005  # Typical value for a water rocket nozzle
drag_coefficient = 0.9  # Based on rocket's shape
air_density = 1.204  # kg/m^3
rocket_radius = 8  # cm for visualization
cross_sectional_area = math.pi * (rocket_radius / 100)**2  # Default area in m^2
nozzle_radius = 1.05  # Example: 1 cm nozzle radius
nozzle_area = math.pi * nozzle_radius**2
water_density = 1000  # kg/m^3
pressure_difference = 100000  # Example pressure difference in Pascals
MAX_MASS = 10.0  # kg
MIN_MASS = 0.01  # kg
MAX_WATER_AMOUNT = 5.0  # L
MIN_WATER_AMOUNT = 0.1  # L

# Rocket image
rocket_image = pygame.image.load("C:/Users/Ari/Downloads/water_rocket/rocket.png")
rocket_image = pygame.transform.scale(rocket_image, (rocket_radius * 2, rocket_radius * 4))  # Resize the rocket

# Global variables for the rocket's parameters
mass = 1.0  # Default mass in kg
water_amount = 1.0  # Default water amount in liters


# Rocket class that will simulate the physics
class WaterRocket:
    def __init__(self):
        self.body = None
        self.launched = False
        self.launch_time = 0
        self.max_height = 0
        self.create_rocket()

    def create_rocket(self):
        if self.body:
            for shape in self.body.shapes:
                space.remove(shape)
            space.remove(self.body)

        self.body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, rocket_radius))
        self.body.position = (width // 2, height // 10)
        shape = pymunk.Circle(self.body, rocket_radius)
        shape.elasticity = 0.4
        space.add(self.body, shape)
        self.max_height = 0

    def reset(self):
        self.create_rocket()  # Recreate the rocket using updated mass and other parameters
        self.launched = False
        self.launch_time = 0
        self.max_height = 0


    def calculate_thrust(self):
        if water_amount <= 0:
            return 0  # No water, no thrust

        # Calculate exhaust velocity
        exhaust_velocity = math.sqrt(2 * pressure_difference / water_density)

        # Calculate mass flow rate
        mass_flow_rate = discharge_coefficient * nozzle_area * math.sqrt(2 * pressure_difference / water_density)

        # Total thrust
        thrust = mass_flow_rate * exhaust_velocity

        print(f"Exhaust Velocity: {exhaust_velocity:.2f} m/s")
        print(f"Mass Flow Rate: {mass_flow_rate:.2f} kg/s")
        print(f"Thrust: {thrust:.2f} N")

        return thrust

    def calculate_drag(self, velocity):
        speed = velocity.length
        max_velocity = 100.0  # Adjust this cap if necessary
        speed = min(speed, max_velocity)
        drag_force = 0.5 * drag_coefficient * air_density * cross_sectional_area * (speed**2)
        return drag_force

    def calculate_real_life_velocity(self):
        if mass <= 0:
            return 0  # Avoid division by zero

        total_mass = mass + (water_amount / 1000.0)  # Convert water volume (L) to mass (kg)

        if total_mass <= 0:
            return 0  # Avoid division by zero

        thrust = self.calculate_thrust()
        velocity = thrust / total_mass  # v = F / m
        return velocity

    def calculate_real_life_height(self, time_passed):
        total_mass = mass + (water_amount / 1000.0)  # Convert water volume to mass
        if total_mass <= 0:
            return 0  # Avoid division by zero

        # Calculate initial velocity
        thrust = self.calculate_thrust()
        initial_velocity = thrust / total_mass

        # Use kinematic equation: h = v0 * t - 0.5 * g * t^2
        g = 9.81  # gravity in m/s^2
        height = initial_velocity * time_passed - 0.5 * g * time_passed**2

        return max(height, 0)  # Ensure height is non-negative

rocket = WaterRocket()

# GUI using Tkinter
def start_gui():
    global mass, water_amount

    def launch_rocket():
        update_parameters()
        if rocket.launched:
            return

        # Recreate the rocket with updated parameters
        rocket.create_rocket()

        # Calculate initial velocity based on thrust
        thrust_velocity = rocket.calculate_real_life_velocity()
        rocket.body.velocity = pymunk.Vec2d(0, thrust_velocity)

        rocket.launched = True
        rocket.launch_time = time.time()
        launch_button.config(state=tk.DISABLED)
        reset_button.config(state=tk.NORMAL)

    def calculate_and_update():
        update_parameters()  # Ensure the parameters are up-to-date from the GUI

        # Calculate velocity and height using the updated parameters
        velocity = rocket.calculate_real_life_velocity()
        height = rocket.calculate_real_life_height(time_passed=1.0)  # Assuming 1 second of thrust for estimation

        # Update GUI labels
        velocity_label.config(text=f"Estimated Velocity: {velocity:.2f} m/s")
        max_height_label.config(text=f"Estimated Height: {height:.2f} m")

    def update_parameters():
        global mass, water_amount
        try:
            mass_val = float(mass_entry.get())
            water_val = float(water_entry.get())
            if not (MIN_MASS <= mass_val <= MAX_MASS):
                error_label.config(text=f"Mass must be between {MIN_MASS} and {MAX_MASS} kg")
                return
            if not (MIN_WATER_AMOUNT <= water_val <= MAX_WATER_AMOUNT):
                error_label.config(text=f"Water amount must be between {MIN_WATER_AMOUNT} and {MAX_WATER_AMOUNT} L")
                return
            # Update global variables
            mass = mass_val
            water_amount = water_val

            # Debug: Confirm updated parameters
            print(f"Updated Parameters:")
            print(f"  Mass: {mass:.2f} kg")
            print(f"  Water Amount: {water_amount:.2f} L")

            # Recreate the rocket with updated parameters
            rocket.reset()
            error_label.config(text="")
        except ValueError:
            error_label.config(text="Invalid input. Please enter numeric values.")

    def reset_simulation():
        rocket.reset()
        launch_button.config(state=tk.NORMAL)
        reset_button.config(state=tk.DISABLED)
        error_label.config(text="")
        velocity_label.config(text="Velocity: 0.00 m/s")
        max_height_label.config(text="Max Height: 0.00 m")

    # GUI Layout
    root = tk.Tk()
    root.title("Water Rocket Simulator")

    frame = ttk.Frame(root, padding=10)
    frame.grid(row=0, column=0, sticky=(tk.W, tk.E))

    ttk.Label(frame, text="Mass (kg):").grid(row=0, column=0, padx=5, pady=5)
    mass_entry = ttk.Entry(frame)
    mass_entry.insert(0, str(mass))
    mass_entry.grid(row=0, column=1, padx=5, pady=5)

    ttk.Label(frame, text="Water Amount (L):").grid(row=1, column=0, padx=5, pady=5)
    water_entry = ttk.Entry(frame)
    water_entry.insert(0, str(water_amount))
    water_entry.grid(row=1, column=1, padx=5, pady=5)

    velocity_label = ttk.Label(frame, text="Estimated Velocity: 0.00 m/s")
    velocity_label.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

    max_height_label = ttk.Label(frame, text="Estimated Height: 0.00 m")
    max_height_label.grid(row=3, column=0, columnspan=2, padx=5, pady=5)

    launch_button = ttk.Button(frame, text="Launch", command=launch_rocket)
    launch_button.grid(row=4, column=0, padx=5, pady=5)

    reset_button = ttk.Button(frame, text="Reset", command=reset_simulation, state=tk.DISABLED)
    reset_button.grid(row=4, column=1, padx=5, pady=5)

    calculate_button = ttk.Button(frame, text="Calculate", command=calculate_and_update)
    calculate_button.grid(row=4, column=2, padx=5, pady=5)

    error_label = ttk.Label(frame, text="", foreground="red")
    error_label.grid(row=5, column=0, columnspan=2, padx=5, pady=5)

    root.mainloop()



def run_simulation():
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        if rocket.launched:  # Update physics only when the rocket is launched
            current_time = time.time()

            # Apply thrust for a limited time (e.g., 1 second)
            if current_time - rocket.launch_time <= 1.0:  # Thrust duration
                thrust_force = pymunk.Vec2d(0, -rocket.calculate_thrust())
                rocket.body.apply_force_at_local_point(thrust_force, (0, 0))

            # After thrust is no longer applied, rely on drag and gravity to slow the rocket
            if current_time - rocket.launch_time > 1.0:
                # Apply drag force (which will slow the rocket down)
                drag_force = -rocket.body.velocity.normalized() * rocket.calculate_drag(rocket.body.velocity)
                rocket.body.apply_force_at_local_point(drag_force, (0, 0))

            # Apply gravity force (downward force)
            gravity_force = pymunk.Vec2d(0, -rocket.body.mass * space.gravity[1])
            rocket.body.apply_force_at_local_point(gravity_force, (0, 0))

        # Step the physics simulation
        if rocket.launched or rocket.body.velocity.length > 0:
            space.step(1 / 60.0)

        # Render the scene
        screen.fill((255, 255, 255))
        draw_rocket()
        pygame.display.flip()
        clock.tick(60)
        
def draw_rocket():
    x, y = rocket.body.position
    y = height - y  # Convert PyMunk's coordinates to Pygame's (flipping y-axis)
    screen.blit(rocket_image, (x - rocket_radius, y - rocket_radius * 2))

run_simulation_thread = Thread(target=run_simulation, daemon=True)
run_simulation_thread.start()
start_gui()  # Start the Tkinter GUI
