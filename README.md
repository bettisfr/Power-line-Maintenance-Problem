# Problema delle Consegne con Droni in Cooperazione con un truck ed Estese Capacità di Carico

## Sketch del problema (instabile)
un truck trasporta una flotta di droni, e abbiamo un insieme di consegne da svolgere.
Per ogni consegna sappiamo la traiettoria più "intelligente" per servire il cliente e per tornare sul truck.
Ogni traiettoria può essere rappresentata come un intervallo.
Ogni intervallo è associato ad un costo energetico e un guadagno.
Ogni drone ha una capacità di carico fissata n e può pertanto deconflittualizzare n consegne contigue.
Il problema ci richiede di trovare per ogni drone il sottoinsieme di intervalli (compatibili fra di loro) che danno il massimo guadagno e il cui costo energetico sia <= alla batteria di ogni drone.
