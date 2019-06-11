#!/usr/bin/awk -f
# $Id: gencfg.awk,v 1.9 2017/10/29 16:26:25 andreas Exp $

# Diese Skript zeigt, wie man aus einem Konfigurationsfile die Spezifikation
# der Tastatur auslesen und daraus eigene Bewertungskriterien erstellen kann.

# This script demonstrates how to extract the specification of a keyboard
# layout from a configuration file, and to create custom evaluation criteria
# from it.

# Aufruf/Invocation:
#
#   ./gencfg.awk standard.cfg > rolle.cfg
#
# oder/or
#
#   awk -f gencfg.awk standard.cfg > rolle.cfg


function abs(a){
   if(a < 0){ return -a; }
   return a;
}

# 'extra' ist der Aufwand für Nicht-Rollen/'extra' is the effort for non-rolls.
# 'aus' ist der Aufwand für Auswärtsrolle/'aus' is the effort for outward rolls.
# 'ein' ist der Aufwand für Einwärtsrolle/'ein' is the effort for inward rolls.

BEGIN {
   extra = 2;
   aus = 0;
   ein = 0;
   n = 0;
}


# Kopie der Eingabekonfiguration durchreichen/pass through a copy of the input
# configuration file

{ print }


# Tastenfestlegungen herausklauben/Pick up definition of keys

/^(Taste)|^(ShiftL)|^(ShiftR)/ {
   name[n]     = $2;
   spalte[n]   = $3;    column[n] = $3;
   zeile[n]    = $4;    row[n]    = $4;
   x[n]        = $5;
   y[n]        = $6;
   finger[n]   = $7;
   grundpos[n] = $8;    restpos[n] = $8;
   aufwand[n]  = $9;    effort[n]  = $9;

   shiftseite[n] = finger[n] < 0 ? 1 : (finger[n] > 0 ? -1 : 0);
   if($10 == "+"){
      shiftseite[n] = -1;
   }else if($10 == "-"){
      shiftseite[n] = 1;
   }
   shiftside[n] = shiftseite[n];

   n++;
}


# Zusätzliche Bewertungskriterien anhängen/Append additional evaluation
# criteria.

END {
   for(i = 0; i < n; ++i){
      for(j = 0; j < n; ++j){
         # Bigramme: Als Beispiel behandeln wir Rollen, Bigramme von Tasten
         # derselben Zeile, die von benachbarten Fingern angeschlagen werden.
         # Rollen bekommen einen Namen und werden so in der Textausgabe
         # aufgeführt; andere Bigramme bekommen einen Zusatzaufwand.

         # Digrams: As an example, we handle rolls, digrams of keys from the
         # same row, which are struck by adjacent fingers.  Rolls get a name
         # and, thereby, are accounted for in the text output; other digrams
         # get an additional effort.

         if(zeile[i] == zeile[j] &&          # gleiche Zeile/same row
            abs(finger[i]-finger[j]) == 1){  # Nachbarn/adjacent
            if(abs(finger[i]) > abs(finger[j])){
               # inward roll
               print "Bigramm", name[i], name[j], ein, "'Einwärtsrolle'"
            }else{
               # outward roll
               print "Bigramm", name[i], name[j], aus, "'Auswärtsrolle'"
            }
         }else{
            # Zusatzaufwand/additional effort
            if(extra != 0){
               print "Bigramm", name[i], name[j], extra
            }
         }

         # for(k = 0; k < n; ++k){
         #     # Trigramme/trigrams
         # }
      }
   }
}
