# HDD Planner Version 2.2 Updated

class HDDPlanner:
    def __init__(self):
        self.max_bend = 0.1  # Corrected max bend to 10%
        self.glossary = {
            "Bend Radius": "The minimum radius a cable can be bent without compromising its performance.",
            "Catenary": "The curve a hanging cable or chain forms under its own weight.",
            "Drafting": "The process of creating technical drawings to define a design's geometry.",
            "Excavation": "The process of removing earth to create a space for laying infrastructure.",
            "Furnishings": "The necessary items and equipment that will fill a space.",
            "Laying": "The act of placing cables, pipes, or other infrastructure into an excavated area.",
            "Right of Way": "The legal right to pass through property owned by another.",
            "Tension": "The force that is exerted in a cable or line that is being pulled.",
            "Trenching": "Creating a narrow excavation in the ground, often used for laying utilities.",
            "Subsurface": "The layer of material beneath the surface of the ground.",
            "Load Bearing": "Referring to the capacity of a structure to hold weight.",
            "Installation": "The process of setting something in place for use.",
            "Surveying": "The process of measuring distances and angles on the earth's surface.",
            " utility": "Services provided to the public such as water, electricity, and telecommunications.",
            "Site Plan": "A detailed drawing that outlines the existing and proposed conditions of a site."
        }

    def calculate(self, length, height):
        # Sample calculation method that maintains existing functionality
        return length * height * self.max_bend

    def display_glossary(self):
        for term, definition in self.glossary.items():
            print(f"{term}: {definition}")

# Example usage
if __name__ == "__main__":
    planner = HDDPlanner()
    planner.display_glossary()