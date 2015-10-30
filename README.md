### Robust inferences of travel paths from GPS trajectories

#### How to compile and run it?

In Terminal

Compile:
```
>>> ant
```

Run:
```
>>> java -cp lib/algs4.jar:PathInference.jar -Xmx4g pathinfer.main.PathInference
```

Clean:
```
>>> ant clean
```

The map data under the "map-data" is extracted from the OpenStreetMap and I 
used my [OSM-parser](https://github.com/HengfengLi/osm-parser) to transform OSM to a usable graph structure. And ten 
GPS trace samples are used from the data set used in ["The Geolink Taxi Service Prediction 
Challenge"](http://archive.ics.uci.edu/ml/datasets/Taxi+Service+Trajectory+-+Prediction+Challenge,+ECML+PKDD+2015)
of ECML/PKDD 2015. 

#### How to cite our work? 

Please cite [our paper](http://www.tandfonline.com/doi/abs/10.1080/13658816.2015.1072202?journalCode=tgis20) if you would like to use our code: 

Li, Hengfeng, Lars Kulik, and Kotagiri Ramamohanarao. "Robust inferences of 
travel paths from GPS trajectories." International Journal of Geographical 
Information Science (2015): 1-29. 

You can freely modify the code and share to others, but the code is only permitted for academic 
research use (non-commercial use). 

If you have any question, please email me 
([henli@student.unimelb.edu.au](henli@student.unimelb.edu.au)). 

