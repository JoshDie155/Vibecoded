# PartJoin

Mehrere 3D-Modelle, die eigentlich zusammengehören (z. B. ein in Teile zerlegter Download),
**über markierte Punkte automatisch passgenau zu einem Modell verbinden** – damit man nach dem
3D-Druck nichts mehr zusammensetzen und kleben muss.

Läuft komplett im Browser, keine Installation, keine Daten verlassen den Rechner.

## So funktioniert's

Im Programm zeigt **?** (Knopf oben im 3D-Fenster oder Taste ?) eine Kurzanleitung mit allen Tastenkürzeln
und Mausaktionen.

1. **Modelle laden** – Dateien ins Fenster ziehen (STL, OBJ mit MTL, 3MF, STEP, IGES, BREP, PLY, AMF, GLB/GLTF, FBX,
   DAE, WRL, 3DS, VTK). CAD-Dateien (STEP/IGES/BREP) liest OpenCascade im Hintergrund und trianguliert sie
   fein für den Druck (Sehnenfehler 0,02 mm); jeder Körper wird ein Teil mit Name und Farbe aus der Datei,
   Körper einer Unterbaugruppe bilden eine Gruppe.
   Mehrteilige Dateien werden in einzelne Teile mit ihrer Farbe zerlegt – auch 3MF-Projekte aus
   Bambu Studio, OrcaSlicer und PrusaSlicer (Farbe aus der Filament-Zuordnung). Teile eines Objekts
   bilden eine Gruppe (⛓) und bewegen sich gemeinsam; jedes Teil lässt sich aus seiner Gruppe lösen
   und per Farbwähler umfärben. Liegen die Teile in den Dateien schon richtig zueinander,
   „Lage aus der Datei übernehmen" anhaken.
   **Fester Boden** („Auf die Druckplatte stellen", Standard): Jede Baugruppe – alles, was über
   Verbindungen oder Gruppen zusammenhängt – steht mit ihrem tiefsten Punkt auf der Druckplatte,
   egal welches Teil die Basis ist. So schwebt im Slicer nichts, obwohl der Export ein einziges
   Objekt ist. Teile, die mit „Lage aus der Datei übernehmen" geladen wurden, stehen gemeinsam auf der
   Platte und bleiben untereinander, wie sie in den Dateien liegen (z. B. Kopf über Körper); ebenso die
   Körper einer CAD-Baugruppe und die Netze einer mehrteiligen OBJ-, GLB- oder FBX-Datei. Das Gizmo
   verschiebt Teile dann nur über die Platte; zum freien Anheben den Haken entfernen – alles bleibt
   dabei, wo es steht.
   Aus Slicer-Projekten kommen auch **Aufschriften und Logos als Modifier** (z. B. eingelassener Text
   in anderem Filament), **Farbbemalung**, Fuzzy-Skin-, Stützen- und Naht-Bemalung sowie die
   Druckeinstellungen je Teil mit: Sie werden angezeigt (die Oberfläche erscheint dort in der
   Filamentfarbe, wo sie im Modifier liegt – so wie gedruckt) und beim Export nach Bambu Studio
   wieder mitgeschrieben.
   **Projekt speichern/öffnen**: „Projekt speichern" (Strg/⌘+S) legt alle Teile mit Lage, Farben,
   Gruppen, Aufschriften, Bemalung und allen Verbindungen in einer `.partjoin`-Datei ab; „Projekt
   öffnen" oder Hineinziehen macht dort weiter (die vorige Arbeit holt Strg+Z zurück). Zusätzlich
   sichert PartJoin die Sitzung laufend im Browser – nach einem Neuladen bietet es an, den letzten
   Stand wiederherzustellen.
2. **Automatisch oder von Hand auswählen** – **„Alles automatisch verbinden"** sucht für alle Teile
   passende Flächen (deckungsgleiche Schnittflächen, Zapfen und Bohrungen) und setzt sie ab dem
   Basisteil zusammen; **„Vorschlagen"** macht das für eine einzelne Verbindung, „Nächster Vorschlag"
   probiert weitere. Von Hand: oben im 3D-Fenster zwischen **Flächen** (Taste F, Standard), **Kanten** (Taste K),
   **Punkte** (Taste P) und **Pinsel** (Taste B) wechseln.
   - *Flächen:* die Kontaktfläche auf beiden Teilen anklicken. PartJoin erkennt selbst, ob es eine
     ebene Fläche, eine Kreisfläche oder ein Zylinder (Zapfen bzw. Bohrung) ist, und setzt die Teile
     wie in einem CAD-Programm aufeinander: Ebene auf Ebene, Kreis mittig auf Kreis, Zapfen koaxial
     in Bohrung. Mehrere Flächenpaare (z. B. Auflagefläche + Zapfen) legen die Lage vollständig fest.
   - *Punkte:* auf beiden Teilen dieselben markanten Stellen anklicken (Ecken, Kanten …).
     **Die Reihenfolge ist egal** – PartJoin erkennt selbst, welche Markierung zu welcher gehört.
   - *Kanten:* gerade Kante auf gerade Kante (z. B. bündig an einer Außenkante) – gut kombinierbar mit einer Fläche.
   - *Pinsel:* mit gedrückter Maustaste über die Kontaktstelle streichen; beim Loslassen wird daraus eine
     Fläche (eben oder Freiform). Praktisch, wo die automatische Flächenerkennung nicht passt.
   - *Flächen zusammenfassen:* in der Flächenliste zwei oder mehr Flächen anhaken und „… zu einer
     zusammenfassen" klicken – oder mit Alt/⌥ + Klick eine Fläche direkt an die zuletzt gewählte anhängen.
     Das geht mit ebenen, runden und Freiformflächen (nicht mit Kanten) und auch über fest verbundene Teile
     hinweg (z. B. ein Aussparungsgrund, der halb zu den Haaren und halb zum Kopf gehört). Liegt alles auf
     einem Zylinder (etwa eine Rundung, deren grobe Facette einzeln erkannt wurde), bleibt das Ergebnis
     eine runde Fläche; „⇹" trennt sie wieder.
   Punkte, Kanten und Flächen lassen sich auch kombinieren. Besteht das Gegenstück aus mehreren Teilen (z. B.
   eine Maske, die auf Kopf *und* Haare passt), dürfen Flächen auch auf Nachbarteilen liegen, die schon fest mit
   Teil 1 verbunden sind – PartJoin rechnet und prüft den Kontakt dann mit allen diesen Teilen.
3. **Verbinden** – das bewegliche Teil wird exakt an das feste angesetzt. Was die Auswahl offen lässt
   (z. B. die Drehung um einen Zapfen), sucht *Automatisch einpassen* über Flächenkontakt ohne
   Durchdringung: Teile mit Abflachung, Nut oder Lochbild drehen sich so von selbst richtig, und ein
   Zapfen wird bis zum Anschlag eingeschoben. Zwei etwa gleich große ebene Flächen (Schnittflächen,
   Zapfen-Stirn auf Aussparung) kommen mittig aufeinander; gesucht wird dann nur noch die Drehung.
   Passt eine andere Drehung genauso gut (z. B. 180° bei rechteckigen Zapfen), wechselt
   „Um 180° drehen" mit einem Klick.
   Kommt zu einer fertigen Verbindung eine weitere Fläche oder ein Punkt hinzu, bleibt das Teil an
   seiner Stelle, bis auch das Gegenstück gewählt ist.
   *Von Hand anpassen* blendet ein Gizmo ein (volle Drehringe, Pfeile zum Verschieben). Standardmäßig
   lassen sich nur die Richtungen greifen, die noch frei sind (z. B. nur der Drehring um die
   Zapfenachse) – gesperrte Achsen erscheinen als graue gestrichelte Linie. Winkelraster 1°/5°/15°/45°/90°.
   *Feinausrichtung* zieht die Kontaktflächen genau aufeinander, *Einsinken* schiebt die Teile minimal
   ineinander, damit der Slicer sie sicher verschmilzt.
4. **Prüfen** – „Passung prüfen" färbt ein, wo sich die Teile berühren (grün), wo ein Spalt bleibt
   (gelb, z. B. das Spiel eines Zapfens) und wo sie sich durchdringen (rot), mit Maßen je Teilepaar.
   **Messen** (oben „Messen", Taste M): zwei Stellen anklicken ergibt Abstand und ΔX/ΔY/ΔZ, ein Klick auf eine
   Rundung ihren Durchmesser, zwei parallele Flächen ihren Abstand (Wandstärke, Spalt), zwei schräge Flächen
   ihren Winkel (mit Innenwinkel), zwei Zapfen/Bohrungen den Winkel ihrer Achsen, Achse und Fläche den Winkel
   dazwischen. Ecken rasten ein.
   Die **Statuszeile** unten zeigt Teile, Dreiecke, Verbindungen und die Maße der Baugruppe unter dem Zeiger.
   **Druckplatte**: Der Umriss der Platte des gewählten Bambu-Druckers liegt im Ursprung; rot, wenn eine
   Baugruppe daneben liegt oder zu groß ist (mit Hinweis in der Statuszeile). „Auf der Platte anordnen"
   (Taste A) legt alle sichtbaren Baugruppen platzsparend darauf; passt nicht alles auf eine Platte, füllt es
   weitere Platten (rechts daneben gezeichnet, „Platte 2", …). Liegt alles auf den Platten, übernimmt der
   Bambu-Export genau diese Lage – mit derselben Aufteilung auf Platten in Bambu Studio.
   **Druckausrichtung**: „Für den Druck ausrichten" legt jede Baugruppe mit ihrer größten ebenen
   Auflage auf die Druckplatte (bei fast gleicher Fläche die stabilere Lage mit tieferem Schwerpunkt);
   was schon gut liegt, bleibt. Von Hand: oben „Auf Fläche legen" (Taste L), dann die Fläche
   anklicken, die unten liegen soll – die ganze Baugruppe dreht sich mit. Strg+Z macht beides rückgängig.
   **Passung**: oben „Passung" (Taste T), dann einen Zapfen, eine Bohrung oder eine ebene Fläche
   anklicken und das neue Maß eingeben – z. B. Zapfen 0,2 mm dünner (Schnellknöpfe −0,1/−0,2/−0,3),
   Bohrung weiter oder eine Fläche zurücksetzen. Nur die Form ändert sich, die Verbindungen bleiben,
   wo sie sind; so entsteht genau dieser Spalt (gut für getrennt gedruckte Steckteile oder bewegliche
   Gelenke). Strg+Z macht es rückgängig.
5. **Exportieren**
   - **3MF für Bambu Studio / OrcaSlicer** (Projektformat von Bambu): **jede Baugruppe ein eigenes
     Objekt** aus ihren Teilen (mit eigenem Namen), die Objekte liegen zueinander wie in PartJoin und
     lassen sich im Slicer einzeln anordnen („Anordnen", Taste A); jede Teilefarbe wird ein Filament in
     genau dieser Farbe. Die Datei bringt
     Projekteinstellungen für den gewählten Drucker mit (System-Profile von Bambu Studio: 0,4-mm-Düse,
     0,20 mm Standard, Bambu PLA Basic; auch H2C, H2D, H2D Pro und X2D mit zwei Düsen) – ohne sie lädt
     Bambu Studio ab 2.8.2 nur die Geometrie.
     Der Drucker eines geladenen Bambu-Projekts wird vorausgewählt.
     Modifier (Aufschriften, negative Volumen, Stützen-Volumen), Bemalung und Druckeinstellungen je
     Teil (Wände, Infill, Muster …) bleiben erhalten; Texte aus Bambu Studio bleiben dort bearbeitbar
     (Textangaben, eigenes Netz und Lage werden mitgeschrieben); die Filamentnummern werden dabei auf die
     Filamente des Exports umgestellt. Einstellungen, die nur für ein ganzes Objekt gelten (z. B.
     Schichthöhe, Brim), bleiben erhalten, solange alle Teile eines Objekts dieselben Werte mitbringen;
     sonst gilt das Druckprofil.
   - **3MF für PrusaSlicer**: ein Objekt aus mehreren Teilen; Teile gleicher Farbe bekommen dieselbe
     Extrudernummer – im Slicer jeder Nummer das passende Filament zuordnen. Die Bemalung kommt mit,
     Modifier nicht (andere Programme würden sie als Teil des Körpers drucken).
   - **STL, OBJ, 3MF (ein Körper)**: mit „Zu einem Körper vereinigen" werden die Teile per
     boolescher Vereinigung zu einem geschlossenen Netz (Überlappungen und Innenflächen verschwinden).
     Nicht geschlossene Teile werden unverändert angehängt und genannt.
   - **Vorschaubild**: Jede 3MF-Datei bekommt ein Bild der Teile (schräg von vorne, ohne Hintergrund) – der
     Bambu-Export eines je Druckplatte, wie Bambu Studio es in seiner Plattenliste und Dateivorschau zeigt.

Mehr als zwei Teile: einfach weitere Verbindungen anlegen. Alles wird ausgehend vom **Basisteil**
zusammengesetzt (Basis in der Teileliste wählbar).

Bedienung: Linksklick wählt aus (nochmal klicken hebt eine Fläche wieder auf) · Rechtsklick auf einen
Punkt löscht ihn · Strg/⌘+Z nimmt die letzte Aktion zurück (Auswahl, Verbinden, Bewegen,
Ausblenden, Entfernen …), Strg/⌘+Umschalt+Z oder Strg+Y stellt sie wieder her · P/F wechselt Punkte/Flächen ·
im Gizmo G = verschieben, R = drehen, Esc = fertig · linke Maustaste dreht die Ansicht, mittlere oder
rechte verschiebt sie, Mausrad zoomt zum Mauszeiger · Strg/⌘ + Ziehen schiebt ein Teil (mit allem, was
daran hängt) über die Bauplatte. Nach „Verbinden" bleibt die Ansicht, wie sie ist.
**Überhänge & Wände** (Abschnitt 3 oder Taste U): färbt Flächen rot, die zu flach nach unten zeigen und
Stützen brauchen (ab einstellbarem Winkel von der Senkrechten, Standard 45° – auf der Platte liegende und
innere Flächen zählen nicht), und lila, wo eine Wand dünner ist als eingestellt (Standard 0,8 mm = zwei
Bahnen bei 0,4-mm-Düse). Die Flächen stehen in der Leiste; nach dem Drehen („Auf Fläche legen", „Für den
Druck ausrichten") rechnet die Ansicht sofort neu.
**Explosionsansicht** (oben „Explosion", Taste X): zieht jede Baugruppe auseinander, damit man sieht, wie die
Teile zusammengehören – Regler unten rechts für den Abstand. Nur zum Ansehen: Lage, Export und Prüfungen
bleiben unverändert, Markierungen und Anklicken ruhen, bis sie wieder zu ist.
**Schnittansicht** (oben „Schnitt", Taste C): schneidet alles entlang X, Y oder Z auf – Lage per Regler unten
rechts, „⇄ Seite" zeigt die andere Hälfte. Wo die Ebene durch Material geht, erscheint eine Schnittfläche in
einem dunkleren Ton des Teils (auch bei aufeinanderliegenden Teilen sauber getrennt), so lassen sich
Wandstärken, Bohrungen und Passungen ansehen. Klicken und Messen treffen nur, was sichtbar ist – z. B. eine
Bohrung im Inneren.
**Netz reparieren**: Nach dem Laden prüft PartJoin jedes Netz im Hintergrund. Offene oder verdrehte Netze
bekommen ein ⚠ in der Teileliste (und einen Hinweis in der Statuszeile); ein Klick darauf oder Rechtsklick ›
„Netz reparieren" verschweißt fast deckungsgleiche Punkte (feine Risse), entfernt doppelte Dreiecke und
Innenwände, gleicht den Umlaufsinn an, dreht nach innen zeigende Netze nach außen und schließt Löcher
(ebene Ränder sauber abgetragen, verwinkelte mit einem Fächer). Gewählte Flächen und Bemalung bleiben
erhalten. Zerschneiden und Vereinigen brauchen ein geschlossenes Netz – beim Zerschneiden gibt es dafür
direkt einen Knopf. Zusammengefallene Dreiecke ohne Fläche gelten nicht als Fehler.
**Toleranz-Test** (links unter den Beispielen): erzeugt eine Platte mit Löchern in steigendem Spiel (0,1 bis
0,4 mm, beschriftet) und einen Zapfen mit Griff – beides drucken, den Zapfen der Reihe nach in die Löcher
stecken; das erste Loch, in das er ohne Gewalt ganz hineingeht, nennt das passende Spiel für deinen Drucker.
Unter „Mein Spiel" gespeichert, nehmen Zerschneiden und Zerteilen es als Standard.
**Aussparungen** (Rechtsklick › „Aussparung hier …" oder Taste E und auf die Stelle klicken): Tasche für einen
Scheibenmagneten (5 × 2 bis 20 × 3 mm), Loch für einen Gewindeeinsatz zum Einschmelzen (M2–M5, übliche
Maße – mit dem Datenblatt abgleichen), Schraubenloch mit Zylinder- oder Senkkopf (M2–M6), Sechskant-Tasche
für eine Mutter (mit Schraubenloch) oder eine freie Bohrung – alle Maße änderbar, mit Spiel. Die Aussparung
steht senkrecht auf der Fläche; auf runden ebenen Flächen sitzt sie in der Mitte. Eine Vorschau zeigt sie, dazu
die Materialstärke dahinter und eine Warnung, wenn sie durchgehen würde. Danach die nächste Stelle anklicken
– z. B. vier Magnete in die Ecken. Gewählte Flächen und Bemalung bleiben erhalten.
**Auf Druckplattengröße zerteilen**: Ist ein Teil zu groß für den gewählten Drucker, bietet die Statuszeile
„Zerteilen …" an (auch per Rechtsklick). PartJoin schlägt vor, wie viele gleich große Stücke je Achse nötig
sind, damit jedes mit Rand für die Verbinder auf die Platte passt (auch um 90° gedreht); mit − / + lässt sich
das ändern, die Schnittebenen sind rot vorgezeigt. Verbinder: Dübel (Standard – auch an senkrechten
Schnitten gut druckbar), Zapfen oder ohne. Danach liegen die Stücke auseinandergerückt wie im Ganzen,
höhere Lagen dahinter auf der Platte, die Dübel davor; „Auf der Platte anordnen" verteilt sie.
**Zerschneiden** (Rechtsklick › „Zerschneiden …"): teilt ein Teil an der Schnittebene in zwei – etwa wenn es
nicht auf die Platte passt oder ohne Stützen druckbar werden soll. Die Ebene stellt man wie in der
Schnittansicht ein (X/Y/Z, Regler, Seite); gezeigt wird nur dieses Teil aufgeschnitten. Verbinder: ohne
(zum Kleben), **Zapfen** an der sichtbaren Hälfte mit passender Bohrung in der anderen, **Dübel**
(Bohrungen in beiden Hälften, Dübel als eigene Teile) oder **Schwalbenschwanz** (Feder quer über den
Querschnitt, 15° Flanken, die andere Hälfte wird seitlich aufgeschoben und hält ohne Kleber). Anzahl und ⌀ automatisch oder vorgegeben, Spiel
einstellbar (Standard 0,2 mm). Die Verbinder sitzen nur dort, wo um die Bohrung genug Wand bleibt, und
reichen nie bis auf die andere Seite durch; die Vorschau zeigt sie in der Schnittfläche. Danach liegen die
Hälften mit der Schnittfläche nach unten nebeneinander (Zapfen nach oben), Dübel stehen daneben – alles mit
Strg/⌘+Z zurücknehmbar. Das Teil muss geschlossen (wasserdicht) sein; die Bemalung bleibt (nur feine Bemalung
genau an der Schnittkante entfällt), Modifier entfallen.
**Skalieren und Spiegeln** (Rechtsklick › „Skalieren / Spiegeln …" oder Taste S über einem Teil bzw. für die
Auswahl): Größe in % oder als Breite/Tiefe/Höhe in mm (gleichmäßig), Schnellwahl 50 %, 200 %, Zoll → mm
(× 25,4) und cm → mm; Spiegeln an X, Y oder Z. Wahlweise die ganze Baugruppe oder nur ein Teil – um die
Mitte, danach wieder auf der Platte. Verbindungen bleiben erhalten: Wird alles gemeinsam umgeformt, gilt ihr
Ergebnis unverändert weiter; bei einem einzelnen Teil setzen sich die verbundenen Teile neu daran. Bemalung
und Bambu-Text skalieren mit (der Text bleibt in Bambu Studio bearbeitbar); beim Spiegeln gehen Bemalung
und Bearbeitbarkeit des Textes verloren (die Textform bleibt), PartJoin sagt das vorher.
**Mehrfachauswahl**: In der Teileliste wählt Strg/⌘+Klick einzelne Teile, Umschalt+Klick einen Bereich; im
3D-Fenster Umschalt+Klick, Strg/⌘+A alle sichtbaren, Esc hebt auf. Ausgewählte Teile haben einen Rahmen;
über der Liste (und per Rechtsklick) lassen sie sich **gruppieren** – sie bleiben, wo sie sind, und bewegen
sich danach gemeinsam (auch beim Strg+Ziehen und im Gizmo) –, ausblenden, gemeinsam einfärben oder
entfernen. „Gruppe auflösen" macht alle Teile einer Gruppe wieder einzeln beweglich.
**Genau platzieren**: Im Gizmo (✥) eines frei liegenden Teils stehen Lage (Mitte X/Y, Unterkante Z) und
„Drehen … ° um X/Y/Z" als Zahlen – Wert eingeben, Enter. Die Pfeiltasten verschieben die Auswahl (oder das
Teil im Gizmo) um 1 mm, mit Umschalt um 10 mm, mit Alt/⌥ um 0,1 mm – passend zur Blickrichtung
(rechts ist rechts im Bild); Bild↑/Bild↓ hebt und senkt (bei ausgeschalteter Druckplatte). Eine Tastenfolge
nimmt Strg/⌘+Z in einem Schritt zurück. Mehrere ausgewählte Baugruppen lassen sich über „Ausrichten …"
links/mittig/rechts bzw. vorne/hinten bündig stellen oder gleichmäßig verteilen.
**Teile verrechnen** (mehrere auswählen, dann „Verrechnen …" über der Liste oder Rechtsklick): vereinigen
(ein Körper ohne Überlappungen, die anderen gehen im Ziel auf), die anderen aus einem Teil ausschneiden
(z. B. eine Form als Aussparung; die Werkzeuge werden ausgeblendet, nicht gelöscht) oder nur die
Überschneidung behalten. Ziel ist das zuletzt angeklickte Teil; es behält Lage, Verbindungen, Bemalung und
Modifier. Alle beteiligten Netze müssen geschlossen sein.
**Rechtsklick auf ein Teil** (im 3D-Fenster oder in der Teileliste) öffnet ein Menü: ausblenden, nur dieses
Teil oder diese Baugruppe zeigen, Farbe, umbenennen, als Basis festlegen, in der Liste zeigen, mit der
angeklickten Fläche auf die Platte legen, Passung an dieser Stelle, von Hand bewegen, Baugruppe bewegen,
aus der Gruppe lösen, entfernen – auf einer gewählten Fläche zuerst „Diese Fläche abwählen". Rechtsklick
ins Leere: Ansicht anpassen, alle zeigen, anordnen, Hilfe. Rechts ziehen verschiebt weiterhin die Ansicht.
Der **Ansichtswürfel** oben rechts dreht sich mit: Fläche, Kante oder Ecke anklicken schwenkt die Ansicht
dorthin (Drehpunkt und Abstand bleiben), Ziehen am Würfel dreht sie, ⌂ zeigt die Schrägansicht. Tasten wie
in Bambu Studio: 1 oben, 2 unten, 3 vorne, 4 hinten, 5 links, 6 rechts, 0 schräg.
Namen: Mauszeiger kurz über einem Teil halten zeigt seinen Namen, Shift+Klick wählt es aus und zeigt es in der Teileliste,
Klick auf einen Namen lässt das Teil aufleuchten, Doppelklick benennt es um. Die Seitenleiste lässt sich
am Rand breiter oder schmaler ziehen (Doppelklick: zurücksetzen).
„Alle Teile" über der Teileliste blendet alle Teile aus oder wieder ein, der Pfeil davor klappt die
Liste ein. Ausgeblendete Teile stehen in einem eigenen, einklappbaren Abschnitt; die Auswahllisten der
Verbindungen zeigen nur sichtbare Teile.
Jedes Teil lässt sich über das Pfeilkreuz in der Teileliste frei von Hand bewegen, „Gesamtes Modell bewegen" verschiebt
oder dreht das ganze zusammengesetzte Modell.

## Runde Teile

Bei Kreisen und Zylindern legt die Form die Drehung um die Achse nicht fest. PartJoin geht so vor:

1. Achsen und Mittelpunkte werden exakt ausgerichtet (Zapfen ↔ Bohrung koaxial, Kreisfläche ↔ Kreisfläche
   mittig). Das Spiel zwischen Zapfen und Bohrung wird angezeigt.
2. Die Drehung wird über den ganzen Kreis abgetastet und die Stellung mit dem besten Flächenkontakt ohne
   Durchdringung gewählt. Hat das Teil irgendetwas Unrundes (Abflachung, Nase, Lochbild, Kanten), ist die
   Stellung damit eindeutig.
3. Ist das Teil wirklich rund, sagt PartJoin das – dann ist die Drehung frei: mit dem Gizmo (Drehring um die
   Achse, Raster z. B. 15°) einstellen oder einen zusätzlichen Punkt auf beiden Teilen setzen.

## Wie viele Punkte?

| Punkte | Ergebnis |
| --- | --- |
| 1 | Teile berühren sich am Punkt, Flächen liegen aufeinander. Drehung um die Kontaktachse per Regler. |
| 2 | Lage eindeutig, wenn die Flächennormalen helfen (Standard). |
| 3+ | Am genauesten. Bei symmetrischen Anordnungen (z. B. Quadrat) warnt PartJoin, wenn mehrere Lösungen unterschiedlich aussehen – „Alternative" schaltet durch. |

## Wie es intern funktioniert

- **Starre Ausrichtung**: Horn-Verfahren (Quaternionen) – findet Rotation + Verschiebung mit der
  kleinsten Abweichung zwischen den Punktpaaren, Spiegelungen ausgeschlossen (`src/math/align.ts`).
- **Automatische Zuordnung**: alle Permutationen der Markierungen werden bewertet (bei vielen Punkten
  mit Abstandsprüfung als Vorfilter, da Abstände bei starren Bewegungen gleich bleiben).
  Flächennormalen gehen als Hilfspunkte ein: an einer Kontaktstelle zeigen die Oberflächen
  gegeneinander (`src/math/match.ts`).
- **Symmetrie-Erkennung**: Lösungen, die das Teil geometrisch auf dieselbe Form abbilden
  (z. B. ein um 180° gedrehtes rechteckiges Dach), gelten nicht als Mehrdeutigkeit.
- **Feinausrichtung**: ICP auf Oberflächenstichproben nahe der Markierungen; nur Flächen, die
  gegeneinander zeigen, zählen als Kontakt (`src/math/icp.ts`, BVH via `three-mesh-bvh`).
- **Flächenerkennung**: vom angeklickten Dreieck aus wird die Fläche über Nachbardreiecke erweitert und als
  Ebene (mit Umriss und ggf. Kreis), Zylinder (Achse, Radius, Zapfen/Bohrung, Bogenwinkel) oder Freiform
  erkannt (`src/math/features.ts`).
- **Paarungen**: Flächenpaare werden zu Bedingungen (Ebene auf Ebene, koaxial, Punkt auf Punkt); ein
  Levenberg-Marquardt-Löser mit mehreren Startlagen findet die Lage und bestimmt, welche Bewegungen danach
  noch frei sind (`src/math/mate.ts`, `src/math/constraints.ts`).
- **Automatisch einpassen**: freie Drehungen/Verschiebungen werden abgetastet und nach Kontakt minus
  Durchdringung bewertet (`src/math/scan.ts`), danach nur entlang freier Richtungen genau angelegt
  (`src/math/settle.ts`); gleiche Umrisse werden deckungsgleich gelegt (`src/math/outline.ts`).
- **Vorschläge**: markante Flächen je Teil (`src/math/segment.ts`), Paare über drehungsunabhängige
  Formmerkmale (Fläche, Umfang, Trägheitsradien) bzw. Durchmesser, verifiziert durch Aufeinanderlegen
  und Durchdringungsprüfung (`src/math/suggest.ts`).
- **Prüfansicht**: Abstand und Vorzeichen zur Oberfläche des anderen Teils je (unterteiltem) Dreieck
  (`src/math/check.ts`).
- **Rechnen im Hintergrund**: 3MF lesen und schreiben, Projekte packen und die Suche nach markanten
  Flächen laufen in einem Web Worker (`src/compute/`); lässt der Browser keinen zu, rechnet dieselbe
  Aufgabe im Hauptthread. Nach dem Laden werden die Flächen schon im Hintergrund gesucht, damit
  „Vorschlagen" sofort antwortet.
- **CAD-Import**: `occt-import-js` (OpenCascade als WebAssembly, `src/io/step.ts`); das WebAssembly steckt
  gzip-gepackt in der Datei (`vite.config.ts`) und wird im Hintergrund-Thread entpackt und gestartet.
  OpenCascade und occt-import-js stehen unter der LGPL-2.1 (Lizenztexte in
  `node_modules/occt-import-js/dist/`); wer PartJoin weitergibt, legt diese Hinweise bei.
- **Projekte**: ZIP mit `project.json` und den Netzen als Float32-Daten, Sitzungssicherung in IndexedDB
  (`src/io/project.ts`).
- **3MF** lesen/schreiben inkl. Slicer-Metadaten (`src/io/threemf.ts`), Bemalung im Format des
  TriangleSelector von Bambu/Prusa (`src/io/paint.ts`), Darstellung von Bemalung und Modifiern per
  Stencil-Puffer (`src/decor.ts`), **Vereinigung** mit manifold-3d (`src/io/union.ts`).

## Entwicklung

```bash
npm install
npm run dev            # Entwicklungsserver
npm test               # Unit-Tests der Mathematik
npm run build          # Build nach dist/
npm run build:single   # eine einzige, eigenständige HTML-Datei: dist-single/index.html
```
