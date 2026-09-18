import numpy as np
import xarray as xr
import capytaine as cpt
import trimesh

simple_model = "models/simple_steelhead.stl"
complex_model = 'models/complex_steelhead.stl'

body_1 = cpt.FloatingBody(
            mesh=cpt.load_mesh(simple_model),
            dofs=cpt.rigid_body_dofs(rotation_center=(0, 0, 0)),
        )
# If you have several rigid bodies, copy the code above to define "body_2", "body_3", etc.

all_bodies = body_1  # Replace "body_1" by "body_1 + body_2 + body_3" for multibody problem.

all_bodies = all_bodies.immersed_part()  # if your mesh has panels above the free surface, remove them

# Set up parameters
test_matrix = xr.Dataset({
        "omega": np.linspace(0.1, 2.0, 20),  # Can also specify "freq" (in Hz), "period", "wavelength" or "wavenumber"
        "wave_direction": np.linspace(0, np.pi, 3),
        "radiating_dof": list(all_bodies.dofs),
        "water_depth": [np.inf],
        "rho": [1025],
        })

# Do the resolution
solver = cpt.BEMSolver()
ds = solver.fill_dataset(test_matrix, all_bodies, hydrostatics=False)

# # Export to netcdf file
# cpt.export_dataset("outputs/added_mass.nc", dataset)

# # Load the NetCDF file
# ds = xr.open_dataset("outputs/added_mass.nc")

# Inspect available variables
# print(ds)

# Added mass is usually stored under 'added_mass'
# indexed by (omega, radiating_dof, forced_dof)
added_mass = ds['added_mass']

# Pick frequency index (e.g. 0 for the first frequency)
freq_index = 0

# Extract 6x6 matrix for that frequency
A = added_mass.isel(omega=freq_index).values

# Pretty print
np.set_printoptions(precision=8, suppress=True)
print("Added mass matrix:", A)

mesh = trimesh.load(complex_model)
print("Center of buoyancy (centroid):", mesh.center_mass)
print("Volume:", mesh.volume)

