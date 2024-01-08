# HGS VRPTW-OOH
This project includes a state-of-the-art algorithm for the "Vehicle routing with private and shared delivery locations" as coined by Mancini & Gansterer (2021). In the problem, the VRPTW is extended by the option of out-of-home (OOH) deliveries. Our algorithm is based upon the HGS for the VRPTW developed by Kool et al. (2022) which is an extension of the HGS-CVRP developed by Vidal (2022).

Our extension is explained in the following paper:

## Citation

When using the code or data in this repo, please cite the following work:

```
@misc{dieter2024VRPTW-OOH,
      title={Incentivizing Out-of-home Deliveries}, 
      author={Peter Dieter and Fabian Akkerman and Martijn Mes and Guido Schryen},
      year={2024}
}
```


## To the make the code work

We use `make` to create an executable. To create this executable run:

```
cd source
make clean
make all
```

 * `source/Params.cpp` Set hyperparameters in this file.
 
 * `hgs_solver.py` Execute the code.
 
 
## Contributing

If you have proposed extensions to this codebase, feel free to do a pull request! If you experience issues, feel free to send us an email.

## License
* [MIT license](https://opensource.org/license/mit/)
* Copyright 2023 © [Peter Dieter](https://en.wiwi.uni-paderborn.de/dep3/schryen/team/dieter), [Fabian Akkerman](https://people.utwente.nl/f.r.akkerman), [Martijn Mes](https://www.utwente.nl/en/bms/iebis/staff/mes/), [Guido Schryen](https://en.wiwi.uni-paderborn.de/dep3/schryen/team/schryen)

## Bibliography

### Problem & Benchmarks

Mancini, Simona, and Margaretha Gansterer. "Vehicle routing with private and shared delivery locations." Computers & Operations Research 133 (2021): 105361.

Grabenschweiger, J., Doerner, K.F., Hartl, R.F. et al. The vehicle routing problem with heterogeneous locker boxes. Cent Eur J Oper Res 29, 113–142 (2021).

Buzzega, Giovanni, and Stefano Novellani. "Last mile deliveries with lockers: Formulations and algorithms." Soft Computing 27.18 (2023): 12843-12861.

### HGS Base Code

Vidal, Thibaut. "Hybrid genetic search for the CVRP: Open-source implementation and SWAP* neighborhood." Computers & Operations Research 140 (2022): 105643.

Kool, W., Joep Olde Juninck, Ernst Roos, Kamiel Cornelissen, Pim Agterberg, Jelke van Hoorn, & Thomas Visser (2022). Hybrid Genetic Search for the Vehicle Routing Problem with Time Windows: a High-Performance Implementation. 12th DIMACS Implementation Challenge Workshop.
