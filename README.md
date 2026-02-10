# Selecting_Electric_Flight_Airports

Code accompanying the paper: **"A two-stage optimization approach for selecting electric-flight airports"**.

## Abstract
The decarbonization of short-haul air transport has gained increasing attention, with electric aircraft emerging as a promising alternative to conventional short-haul aviation. However, given the substantial investment anticipated for the necessary infrastructure, a strategic and globally coordinated selection of airports is imperative. The aim of this paper is to address this problem and determine the best possible selection of airports within a given network.

We develop a bilevel mixed-integer optimization model that incorporates airline decision-making in response to infrastructure deployment. The first stage determines which airports to electrify under budget and technical constraints, while the second stage models airline routing choices based on cost-minimization. The problem is reduced to a single-stage mixed-integer programming framework and solved using a cutting-plane procedure together with a column generation approach, in which the pricing problem can be solved using Dijkstra's algorithm.

We tested our algorithm in two infrastructure expansion scenarios in Germany. For each scenario, we consider the results for electric aircraft with ranges of 300 km and 1480 km. The results indicate that the differences in emission savings between the two range scenarios are marginal. Simultaneously, the optimal solutions demonstrate robustness to fluctuations in kerosene and electricity prices. Additionally, based on the given assumptions and parameters, using routes that are partly operated by electric and conventional aircraft does not appear to be economically viable for short-haul air traffic from an airline's perspective.

## About this repository
This repository contains the C++ implementation used to generate the results presented in the paper. All required data is included.

## Build and Run
The CMake project is located in `CODE/` (contains `CMakeLists.txt`).

git clone <REPO_URL>
cd <REPO_NAME>

mkdir -p build
cd build
cmake ../CODE
make

./facilitylocation
