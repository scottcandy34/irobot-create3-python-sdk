from enum import Enum
import math
from irobot_create_msgs.msg import AudioNote
from builtin_interfaces.msg import Duration as AudioDuration

class Articulation(Enum):
    NORMAL = 0
    STACCATO = 1
    LEGATO = 2
    ACCENT = 3
    TENUTO = 4
    MARCATO = 5
    
class Duration(Enum):
    WHOLE = 1
    HALF = 2
    QUARTER = 4
    EIGHTH = 8
    SIXTEENTH = 16
    
class Ratio():
    def __init__(self, numerator: int, denominator):
        self.numerator = numerator
        self.denominator = denominator
    
    def __str__(self):
        return f"{self.numerator}, {self.denominator}"
    
    def sum(self) -> float | int:
        return self.numerator + self.denominator
    
    def fraction(self) -> float:
        return self.numerator / self.denominator
    
    def reciprocal(self) -> float:
        return self.denominator / self.numerator

class Note():
    def __init__(self, pitch: float | int, duration: Duration, articulation: Articulation = Articulation.NORMAL, is_rest: bool = False, dots: int = 0, tuplet_ratio: Ratio = Ratio(1,1), start_time: float = None):
        # cross_rhythm 3:2, Cross rhythms happen when two or more conflicting rhythms play at the same time, like three beats against two (3:2), creating a rich, layered texture.
        # grace_notes, Grace notes are quick, ornamental notes played just before a main note, like a rapid flourish that doesn’t count toward the measure’s total rhythm. For instance, a tiny note before a quarter note might add a decorative touch.
        # is_tied_to_next
        self.pitch = pitch # Hz
        self.articulation = articulation
        self.is_rest = is_rest # bool
        self.duration = duration # beats
        self.dots = dots # Number of dots the Note has. Original × (2 - 1/2^n)
        self.tuplet_ratio = tuplet_ratio # if 3:2, then 3 notes in space of 2 beats
        
        self._start_time = start_time # retains the beat number this note triggers at
        self._swing: float = None
        
    def getBeats(self, one_beat: int) -> float:
        if self._swing:
            return self._swing
        else:
            return (one_beat / self.duration) * (2 - 1 / (2**self.dots)) * self.tuplet_ratio.reciprocal()
    
    def isDottedOrTuplet(self) -> bool:
        return self.dots > 0 or self.tuplet_ratio.reciprocal() > 1
    
    def setSwing(self, ratio: float):
        self._swing = ratio
    
class Measure():
    def __init__(self, notes: list[Note], time_signature: Ratio):
        # metric_modulation, Changes tempo based on a rhythmic ratio (e.g., quarter note becomes the new eighth note).  metric_modulation: "quarter = eighth"
        self.notes = notes
        self.time_signature = time_signature # 3/4, 3 is the number of beats in the measure, while 4 is the note value that represents 1 beat.
        self.total_beats = time_signature.numerator
        
        # Validate total beats against time signature
        total = sum(note.getBeats(self.time_signature.denominator) for note in self.notes)
        if total > self.total_beats:
            raise ValueError(f"Measure exceeds {self.time_signature} beats!")
        elif total < self.total_beats:
            raise ValueError(f"Measure contains not enough beats!")

class Phrase():
    def __init__(self, measures: list[Measure], slur: bool = False, swing: Ratio = None, hemiola: Ratio = None, polyrhythm: Ratio = None):
        # agogic_accent bool, An agogic accent emphasizes a note by slightly lengthening its duration, rather than making it louder.
        self.measures = measures
        self.slur = slur # A curved line indicating smooth connection across multiple notes, often spanning several measures.
        self.swing = swing # Uneven timing of paired notes (e.g., eighth notes played long-short).
        self.hemiola = hemiola # A rhythmic shift (e.g., two groups of three beats become three groups of two).
        self.polyrhythm = polyrhythm # Multiple simultaneous rhythmic patterns (e.g., 3 against 2).
        self.total_beats = sum(measure.total_beats for measure in self.measures)
        
        # Add swing effect
        for measure in self.measures:
            beats = 0.0
            for note in measure.notes:
                beats += note.getBeats(measure.time_signature.denominator)
                if note.duration == (measure.time_signature.denominator * 2) and not note.isDottedOrTuplet():
                    if beats.is_integer():
                        note.setSwing([swing.denominator / swing.sum()]) # set second note in beat
                    else:
                        note.setSwing([swing.numerator / swing.sum()]) # set first note in beat

class Section():
    def __init__(self, name: str, phrases: list[Phrase]):
        # key_signature, adjusts pitches. pitch adjustment - sharpened or flattened. scale definition - pitch structure
        # tempo_change, at measure 10 change to... accelerandos(faster) or ritardandos(slower)
        self.name = name
        self.phrases = phrases
        self.total_beats = sum(phrase.total_beats for phrase in self.phrases)

class Piece():
    def __init__(self, title: str, sections: list[Section]):
        self.title = title
        self.sections = sections
        self.total_beats = sum(section.total_beats for section in self.sections)
        
        
class Beats():
    def __init__(self, tempo: float | int = 240):
        self._quarter = 60 / tempo # 60(s) / tempo(BPM) = time()
        self._eighth = self._quarter / 2
        self._sixteenth = self._eighth / 2
        self._half = self._quarter * 2
        self._whole = self._half * 2
        
    # def get

class Sequencer():
    def __init__(self, piece: Piece, tempo: float | int):
        self.piece = piece
        
        timing = Beats(tempo)
        
        notes: list[AudioNote] = []
        for sections in self.piece.sections:
            for phrase in sections.phrases:
                for measure in phrase.measures:
                    for note in measure.notes:
                        beat = note.getBeats(measure.time_signature.denominator)
                        t = 
                        # frequency = note.getBeats(measure.time_signature.denominator)
                        duration = AudioDuration(sec=int(beat), nanosec=round((beat - int(beat)) * 1000000000))
                        notes += AudioNote(frequency=frequency, max_runtime=duration)
    
    def getNotes(self):
        
        
        
        pass