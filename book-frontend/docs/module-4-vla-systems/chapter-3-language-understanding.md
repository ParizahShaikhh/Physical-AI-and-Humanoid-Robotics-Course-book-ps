# Chapter 3: Language Understanding and Task Decomposition

## Learning Objectives

By the end of this chapter, you will be able to:
- Parse natural language commands to extract actionable tasks and object references
- Implement task decomposition algorithms for complex commands
- Create command-to-action mapping systems for robotic applications
- Handle ambiguous commands with context-based disambiguation
- Integrate language understanding with vision and action components

## Introduction to Language Understanding in Robotics

Language understanding in robotic systems bridges the gap between human natural language commands and executable robot actions. Unlike traditional command-based interfaces, natural language allows for complex, context-dependent instructions that require sophisticated parsing and interpretation.

The challenge in robotic language understanding lies in translating high-level, often ambiguous human commands into precise, executable tasks that account for environmental constraints and physical realities. This requires not only linguistic analysis but also integration with perception and action systems.

## Natural Language Processing for Robotics

### Command Structure Analysis

Robotic commands typically follow specific patterns that can be analyzed for structured extraction:

```
Command: "Go to the kitchen and bring me a red apple"
Components:
- Navigation task: "Go to the kitchen"
- Manipulation task: "bring me a red apple"
- Object specification: "red apple"
- Location: "kitchen"
```

### Linguistic Components

Robotic language understanding typically involves:

1. **Tokenization**: Breaking commands into meaningful units
2. **Part-of-speech tagging**: Identifying nouns, verbs, adjectives
3. **Named entity recognition**: Identifying objects, locations, people
4. **Dependency parsing**: Understanding grammatical relationships
5. **Semantic role labeling**: Identifying action-argument relationships

```python
import spacy
import re
from typing import List, Dict, Tuple

class RobotCommandParser:
    def __init__(self):
        # Load English language model
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            print("Please install spaCy English model: python -m spacy download en_core_web_sm")
            self.nlp = None

    def parse_command(self, command: str) -> Dict:
        """
        Parse a natural language command into structured components
        """
        if not self.nlp:
            return self.fallback_parse(command)

        doc = self.nlp(command)

        # Extract entities
        entities = [(ent.text, ent.label_) for ent in doc.ents]

        # Extract actions (verbs)
        actions = [token.lemma_ for token in doc if token.pos_ == "VERB"]

        # Extract objects (nouns)
        objects = [token.lemma_ for token in doc if token.pos_ in ["NOUN", "PROPN"]]

        # Extract adjectives (descriptors)
        adjectives = [token.lemma_ for token in doc if token.pos_ == "ADJ"]

        return {
            "original_command": command,
            "entities": entities,
            "actions": actions,
            "objects": objects,
            "adjectives": adjectives,
            "tokens": [token.text for token in doc],
            "dependencies": [(token.text, token.dep_, token.head.text) for token in doc]
        }

    def fallback_parse(self, command: str) -> Dict:
        """
        Fallback parsing using regex patterns
        """
        # Simple pattern matching for common command structures
        action_patterns = [
            (r"go to|navigate to|move to", "navigation"),
            (r"pick up|grasp|take|grab", "grasp"),
            (r"bring|deliver|give", "delivery"),
            (r"find|locate|look for", "search"),
            (r"open|close", "manipulation"),
            (r"turn on|turn off", "control")
        ]

        actions = []
        for pattern, action_type in action_patterns:
            if re.search(pattern, command, re.IGNORECASE):
                actions.append(action_type)

        # Extract location references
        location_patterns = [r"to (\w+)", r"at (\w+)", r"in the (\w+)", r"(\w+) room"]
        locations = []
        for pattern in location_patterns:
            matches = re.findall(pattern, command, re.IGNORECASE)
            locations.extend(matches)

        # Extract object references
        object_patterns = [r"(\w+ \w+ apple)|(\w+ apple)", r"(\w+ ball)", r"the (\w+)"]
        objects = []
        for pattern in object_patterns:
            matches = re.findall(pattern, command, re.IGNORECASE)
            for match in matches:
                if isinstance(match, tuple):
                    objects.extend([m for m in match if m])
                else:
                    objects.append(match)

        return {
            "original_command": command,
            "entities": [],
            "actions": actions,
            "objects": objects,
            "adjectives": [],
            "locations": locations,
            "tokens": command.split()
        }
```

## Task Decomposition Algorithms

### Hierarchical Task Network (HTN)

HTN decomposition breaks complex tasks into hierarchical subtasks:

```python
class HTNDecomposer:
    def __init__(self):
        self.primitive_actions = {
            "navigate_to": self.navigate_to,
            "grasp_object": self.grasp_object,
            "place_object": self.place_object,
            "search_object": self.search_object
        }

        self.complex_tasks = {
            "fetch_object": self.decompose_fetch_object,
            "move_object": self.decompose_move_object,
            "prepare_workspace": self.decompose_prepare_workspace
        }

    def decompose_task(self, task: str, params: Dict) -> List[Dict]:
        """
        Decompose a complex task into primitive actions
        """
        if task in self.complex_tasks:
            return self.complex_tasks[task](params)
        elif task in self.primitive_actions:
            return [{"action": task, "params": params}]
        else:
            raise ValueError(f"Unknown task: {task}")

    def decompose_fetch_object(self, params: Dict) -> List[Dict]:
        """
        Decompose 'fetch object' task
        """
        return [
            {"action": "navigate_to", "params": {"location": params.get("start_location")}},
            {"action": "search_object", "params": {
                "object_type": params.get("object_type"),
                "color": params.get("color", "any"),
                "location": params.get("search_location")
            }},
            {"action": "grasp_object", "params": {
                "object_id": params.get("object_id")
            }},
            {"action": "navigate_to", "params": {"location": params.get("delivery_location")}},
            {"action": "place_object", "params": {
                "location": params.get("delivery_location")
            }}
        ]

    def decompose_move_object(self, params: Dict) -> List[Dict]:
        """
        Decompose 'move object' task
        """
        return [
            {"action": "navigate_to", "params": {"location": params.get("start_location")}},
            {"action": "grasp_object", "params": {
                "object_id": params.get("object_id")
            }},
            {"action": "navigate_to", "params": {"location": params.get("end_location")}},
            {"action": "place_object", "params": {
                "location": params.get("end_location")
            }}
        ]

    # Primitive action implementations would go here
    def navigate_to(self, params):
        pass

    def grasp_object(self, params):
        pass

    def place_object(self, params):
        pass

    def search_object(self, params):
        pass
```

### Finite State Machine Approach

For more complex scenarios, a state machine can manage task decomposition:

```python
from enum import Enum
from dataclasses import dataclass
from typing import Optional

class TaskState(Enum):
    PENDING = "pending"
    PROCESSING = "processing"
    SEARCHING = "searching"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class TaskContext:
    command: str
    current_state: TaskState
    subtasks: List[Dict]
    current_subtask_index: int
    objects_found: List[Dict]
    robot_location: str
    error_count: int = 0

class StatefulTaskDecomposer:
    def __init__(self):
        self.hierarchy = HTNDecomposer()

    def process_command(self, command: str, context: TaskContext) -> TaskContext:
        """
        Process command and update task context based on current state
        """
        if context.current_state == TaskState.PENDING:
            # Parse command and decompose into subtasks
            parsed = self.parse_command(command)
            context.subtasks = self.hierarchy.decompose_task(parsed["task"], parsed["params"])
            context.current_state = TaskState.PROCESSING
            context.current_subtask_index = 0

        elif context.current_state == TaskState.PROCESSING:
            # Execute current subtask
            if context.current_subtask_index < len(context.subtasks):
                subtask = context.subtasks[context.current_subtask_index]
                context = self.execute_subtask(subtask, context)

                if context.current_state == TaskState.COMPLETED:
                    context.current_subtask_index += 1
                    if context.current_subtask_index < len(context.subtasks):
                        context.current_state = TaskState.PROCESSING
                    else:
                        context.current_state = TaskState.COMPLETED
            else:
                context.current_state = TaskState.COMPLETED

        return context

    def parse_command(self, command: str) -> Dict:
        """
        Parse command into task and parameters
        """
        parser = RobotCommandParser()
        parsed = parser.parse_command(command)

        # Determine the main task from the command
        actions = parsed["actions"]
        if "grasp" in actions or "take" in actions or "pick" in actions:
            task = "fetch_object"
        elif "go" in actions or "navigate" in actions or "move" in actions:
            task = "navigate"
        else:
            task = "generic_task"  # Default fallback

        return {
            "task": task,
            "params": {
                "command": command,
                "actions": actions,
                "objects": parsed["objects"],
                "locations": [ent[0] for ent in parsed["entities"] if ent[1] in ["GPE", "LOC", "FAC"]]
            }
        }

    def execute_subtask(self, subtask: Dict, context: TaskContext) -> TaskContext:
        """
        Execute a single subtask and update context
        """
        action = subtask["action"]
        params = subtask["params"]

        if action == "navigate_to":
            context.current_state = TaskState.NAVIGATING
            # Implementation would interface with navigation system
            # context.robot_location = params["location"]
            context.current_state = TaskState.COMPLETED
        elif action == "search_object":
            context.current_state = TaskState.SEARCHING
            # Implementation would interface with perception system
            # context.objects_found = search_results
            context.current_state = TaskState.COMPLETED
        elif action == "grasp_object":
            context.current_state = TaskState.MANIPULATING
            # Implementation would interface with manipulation system
            context.current_state = TaskState.COMPLETED

        return context
```

## Command-to-Action Mapping

### Semantic Mapping Framework

Map linguistic elements to robotic actions:

```python
class SemanticMapper:
    def __init__(self):
        self.action_mapping = {
            # Navigation actions
            "go": "navigate_to",
            "move": "navigate_to",
            "navigate": "navigate_to",
            "walk": "navigate_to",
            "travel": "navigate_to",

            # Manipulation actions
            "grasp": "grasp_object",
            "take": "grasp_object",
            "pick": "grasp_object",
            "grab": "grasp_object",
            "lift": "grasp_object",
            "hold": "grasp_object",

            "place": "place_object",
            "put": "place_object",
            "set": "place_object",
            "drop": "place_object",

            # Object search
            "find": "search_object",
            "locate": "search_object",
            "look_for": "search_object",
            "search": "search_object",

            # Complex actions
            "bring": "fetch_object",
            "fetch": "fetch_object",
            "get": "fetch_object",
            "deliver": "deliver_object",
            "give": "deliver_object"
        }

        self.location_mapping = {
            "kitchen": "kitchen_waypoint",
            "living room": "living_room_waypoint",
            "bedroom": "bedroom_waypoint",
            "office": "office_waypoint",
            "dining room": "dining_room_waypoint",
            "bathroom": "bathroom_waypoint",
            "hallway": "hallway_waypoint"
        }

        self.object_mapping = {
            # Food items
            "apple": "food_apple",
            "orange": "food_orange",
            "banana": "food_banana",
            "water": "drink_water",
            "juice": "drink_juice",

            # Household items
            "cup": "object_cup",
            "book": "object_book",
            "phone": "object_phone",
            "keys": "object_keys",
            "bottle": "object_bottle",
            "toy": "object_toy",

            # Furniture
            "table": "furniture_table",
            "chair": "furniture_chair",
            "sofa": "furniture_sofa",
            "bed": "furniture_bed",
            "couch": "furniture_sofa"
        }

    def map_command_to_action(self, parsed_command: Dict) -> List[Dict]:
        """
        Map parsed command to executable actions
        """
        actions = []

        # Map main action
        if parsed_command["actions"]:
            main_action = parsed_command["actions"][0]
            robot_action = self.action_mapping.get(main_action, "unknown_action")

            action_params = self.extract_parameters(parsed_command)

            if robot_action != "unknown_action":
                actions.append({
                    "action": robot_action,
                    "parameters": action_params
                })

        return actions

    def extract_parameters(self, parsed_command: Dict) -> Dict:
        """
        Extract parameters from parsed command
        """
        params = {}

        # Extract location
        for entity, label in parsed_command["entities"]:
            if label in ["GPE", "LOC", "FAC"]:
                params["location"] = self.location_mapping.get(entity.lower(), entity)
                break

        # Extract object
        for obj in parsed_command["objects"]:
            if obj in self.object_mapping:
                params["object"] = self.object_mapping[obj]
                break
            else:
                params["object"] = obj  # Use as-is if not in mapping

        # Extract adjectives (descriptors)
        if parsed_command["adjectives"]:
            params["descriptor"] = parsed_command["adjectives"][0]

        return params
```

## Handling Ambiguous Commands

### Context-Based Disambiguation

Use environmental context to resolve ambiguous references:

```python
class Disambiguator:
    def __init__(self):
        self.context_memory = {}
        self.perception_interface = None  # Interface to vision system

    def resolve_ambiguity(self, command: str, parsed: Dict, context: Dict) -> Dict:
        """
        Resolve ambiguous references using context
        """
        resolved = parsed.copy()

        # Handle ambiguous object references
        if "object" in resolved and resolved["object"] == "it":
            # Resolve "it" based on previous context
            if "last_referenced_object" in context:
                resolved["object"] = context["last_referenced_object"]

        # Handle ambiguous locations
        if "location" in resolved and resolved["location"] in ["here", "there"]:
            if resolved["location"] == "here":
                resolved["location"] = context.get("robot_location", "unknown")
            elif resolved["location"] == "there":
                # Use most salient location in context
                resolved["location"] = self.get_salient_location(context)

        # Use perception to disambiguate object references
        if "object" in resolved and "descriptor" in resolved:
            objects_in_env = self.get_objects_in_environment()
            resolved["object"] = self.find_best_matching_object(
                resolved["object"],
                resolved["descriptor"],
                objects_in_env
            )

        return resolved

    def get_objects_in_environment(self) -> List[Dict]:
        """
        Get objects currently perceived in environment
        """
        if self.perception_interface:
            return self.perception_interface.get_detected_objects()
        else:
            # Return mock data for simulation
            return [
                {"id": "apple_1", "type": "apple", "color": "red", "location": "table"},
                {"id": "apple_2", "type": "apple", "color": "green", "location": "counter"},
                {"id": "cup_1", "type": "cup", "color": "white", "location": "table"}
            ]

    def find_best_matching_object(self, obj_type: str, descriptor: str, objects: List[Dict]) -> str:
        """
        Find the best matching object based on type and descriptor
        """
        candidates = [obj for obj in objects if obj["type"] == obj_type]

        if not candidates:
            return obj_type  # Return as-is if no matches

        if descriptor:
            # Filter by descriptor (e.g., color)
            descriptor_matches = [obj for obj in candidates if obj.get("color") == descriptor]
            if descriptor_matches:
                return descriptor_matches[0]["id"]

        # If multiple candidates and no descriptor, return first or ask for clarification
        if len(candidates) > 1:
            # In a real system, this might trigger a clarification request
            return candidates[0]["id"]  # Return first for simulation

        return candidates[0]["id"]

    def get_salient_location(self, context: Dict) -> str:
        """
        Get the most contextually relevant location
        """
        # This would typically analyze the discourse context
        # For now, return the location with most objects or user location
        return context.get("user_location", "unknown")
```

## Integration with ROS 2

### Action Server Implementation

Create a ROS 2 action server for language understanding:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from vla_language_msgs.action import ParseCommand  # Custom action message

class LanguageUnderstandingServer(Node):
    def __init__(self):
        super().__init__('language_understanding_server')

        # Initialize components
        self.parser = RobotCommandParser()
        self.decomposer = HTNDecomposer()
        self.mapper = SemanticMapper()
        self.disambiguator = Disambiguator()

        # Create action server
        self._action_server = ActionServer(
            self,
            ParseCommand,
            'parse_command',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """
        Execute the command parsing goal
        """
        self.get_logger().info(f'Processing command: {goal_handle.request.command}')

        try:
            # Parse the command
            parsed = self.parser.parse_command(goal_handle.request.command)

            # Resolve ambiguities using context
            context = self.get_context_from_environment()
            resolved = self.disambiguator.resolve_ambiguity(
                goal_handle.request.command, parsed, context
            )

            # Map to robot actions
            actions = self.mapper.map_command_to_action(resolved)

            # Decompose complex tasks
            final_plan = []
            for action in actions:
                if action["action"] in self.decomposer.complex_tasks:
                    subtasks = self.decomposer.decompose_task(
                        action["action"], action["parameters"]
                    )
                    final_plan.extend(subtasks)
                else:
                    final_plan.append(action)

            # Create result
            result = ParseCommand.Result()
            result.success = True
            result.action_plan = self.convert_to_action_messages(final_plan)
            result.original_command = goal_handle.request.command

            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Command parsing failed: {str(e)}')
            result = ParseCommand.Result()
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            return result

    def get_context_from_environment(self) -> Dict:
        """
        Get environmental context for disambiguation
        """
        # This would interface with perception and localization systems
        return {
            "robot_location": "starting_position",
            "user_location": "living_room",
            "objects_in_view": self.get_objects_in_environment()
        }

    def convert_to_action_messages(self, plan) -> List:
        """
        Convert plan to ROS 2 action messages
        """
        # Implementation would convert internal plan representation
        # to appropriate ROS 2 message types
        pass

    def get_objects_in_environment(self):
        """
        Mock implementation - would interface with perception
        """
        return [
            {"id": "red_apple_1", "type": "apple", "color": "red", "location": "table"}
        ]

def main(args=None):
    rclpy.init(args=args)
    server = LanguageUnderstandingServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()
```

## Practical Examples and Use Cases

### Complex Command Processing

Here's a complete example of processing a complex command:

```python
def process_complex_command_example():
    """
    Example: Process "Go to the kitchen and bring me the red apple from the table"
    """
    command = "Go to the kitchen and bring me the red apple from the table"

    # Initialize components
    parser = RobotCommandParser()
    decomposer = HTNDecomposer()
    mapper = SemanticMapper()
    disambiguator = Disambiguator()

    # Step 1: Parse command
    parsed = parser.parse_command(command)
    print("Parsed command:", parsed)

    # Step 2: Resolve ambiguities (in a real system, this would use context)
    context = {
        "robot_location": "living_room",
        "user_location": "living_room",
        "objects_in_view": [
            {"id": "red_apple_1", "type": "apple", "color": "red", "location": "table"}
        ]
    }
    resolved = disambiguator.resolve_ambiguity(command, parsed, context)
    print("Resolved command:", resolved)

    # Step 3: Map to actions
    actions = mapper.map_command_to_action(resolved)
    print("Mapped actions:", actions)

    # Step 4: Decompose complex tasks
    full_plan = []
    for action in actions:
        if action["action"] in decomposer.complex_tasks:
            subtasks = decomposer.decompose_task(action["action"], action["parameters"])
            full_plan.extend(subtasks)
        else:
            full_plan.append(action)

    print("Full execution plan:")
    for i, task in enumerate(full_plan):
        print(f"  {i+1}. {task['action']} with params: {task.get('params', task.get('parameters', {}))}")

# Run the example
if __name__ == "__main__":
    process_complex_command_example()
```

## Error Handling and Fallback Strategies

### Graceful Degradation

Handle various failure modes gracefully:

```python
class RobustLanguageProcessor:
    def __init__(self):
        self.parser = RobotCommandParser()
        self.decomposer = HTNDecomposer()
        self.mapper = SemanticMapper()
        self.disambiguator = Disambiguator()
        self.fallback_strategies = [
            self.try_simplified_parsing,
            self.request_clarification,
            self.use_default_action
        ]

    def process_command_with_fallback(self, command: str, context: Dict) -> Tuple[List[Dict], bool]:
        """
        Process command with fallback strategies
        """
        try:
            # Primary processing
            parsed = self.parser.parse_command(command)
            resolved = self.disambiguator.resolve_ambiguity(command, parsed, context)
            actions = self.mapper.map_command_to_action(resolved)

            # Decompose tasks
            plan = []
            for action in actions:
                if action["action"] in self.decomposer.complex_tasks:
                    subtasks = self.decomposer.decompose_task(action["action"], action["parameters"])
                    plan.extend(subtasks)
                else:
                    plan.append(action)

            return plan, True  # Success

        except Exception as e:
            self.get_logger().warning(f"Primary processing failed: {e}")

            # Try fallback strategies
            for i, fallback in enumerate(self.fallback_strategies):
                try:
                    plan, success = fallback(command, context)
                    if success:
                        self.get_logger().info(f"Fallback strategy {i+1} succeeded")
                        return plan, True
                except Exception as fb_e:
                    self.get_logger().warning(f"Fallback {i+1} failed: {fb_e}")
                    continue

            # All strategies failed
            return [], False

    def try_simplified_parsing(self, command: str, context: Dict) -> Tuple[List[Dict], bool]:
        """
        Fallback: Use simplified parsing
        """
        # Extract basic keywords
        keywords = command.lower().split()

        plan = []
        if any(kw in keywords for kw in ["go", "move", "navigate"]):
            # Find location keyword
            locations = ["kitchen", "bedroom", "office", "living room"]
            for loc in locations:
                if loc in command.lower():
                    plan.append({"action": "navigate_to", "params": {"location": loc}})
                    break

        if any(kw in keywords for kw in ["get", "bring", "fetch", "take"]):
            # Find object keyword
            objects = ["apple", "cup", "book", "water"]
            for obj in objects:
                if obj in command.lower():
                    plan.append({"action": "grasp_object", "params": {"object": obj}})
                    break

        return plan, len(plan) > 0

    def request_clarification(self, command: str, context: Dict) -> Tuple[List[Dict], bool]:
        """
        Fallback: Request clarification from user
        """
        # In a real system, this would publish a request for clarification
        clarification_needed = True
        clarification_request = f"I didn't understand the command '{command}'. Could you please rephrase it?"

        # For simulation, return empty plan indicating need for clarification
        return [{"action": "request_clarification", "params": {"request": clarification_request}}], True

    def use_default_action(self, command: str, context: Dict) -> Tuple[List[Dict], bool]:
        """
        Fallback: Use default action
        """
        return [{"action": "default_response", "params": {"original_command": command}}], True

    def get_logger(self):
        """
        Mock logger for example
        """
        class MockLogger:
            def info(self, msg):
                print(f"INFO: {msg}")
            def warning(self, msg):
                print(f"WARNING: {msg}")
        return MockLogger()
```

## Performance Considerations

### Efficiency Optimization

Optimize language processing for real-time robotic applications:

```python
import time
from functools import lru_cache

class OptimizedLanguageProcessor:
    def __init__(self):
        self.parser = RobotCommandParser()
        self.command_cache = {}
        self.max_cache_size = 100

    @lru_cache(maxsize=100)
    def cached_parse(self, command: str):
        """
        Cached parsing for frequently used commands
        """
        return self.parser.parse_command(command)

    def process_command_with_timing(self, command: str, context: Dict) -> Tuple[List[Dict], float]:
        """
        Process command and return processing time
        """
        start_time = time.time()

        try:
            # Use cached result if available
            if command in self.command_cache:
                parsed_plan = self.command_cache[command]
            else:
                # Parse and cache result
                parsed_plan = self.cached_parse(command)

                # Add to cache if not full
                if len(self.command_cache) < self.max_cache_size:
                    self.command_cache[command] = parsed_plan

            processing_time = time.time() - start_time
            return parsed_plan, processing_time

        except Exception as e:
            processing_time = time.time() - start_time
            raise e
```

## Summary

Language understanding and task decomposition form the cognitive bridge between human commands and robotic actions. By combining natural language processing, semantic mapping, and task decomposition algorithms, robots can interpret complex, ambiguous commands and execute them as sequences of primitive actions.

The key to successful implementation lies in:
1. Robust parsing that handles linguistic variation
2. Effective disambiguation using environmental context
3. Hierarchical task decomposition for complex commands
4. Graceful error handling and fallback strategies
5. Integration with perception and action systems

## Exercises

1. Implement a command parser that can handle the command "Bring me the red cup from the kitchen table"
2. Create a task decomposer for a "set the table" command that involves multiple objects and locations
3. Add context-aware disambiguation to handle references like "that one" or "the other one"
4. Implement a fallback mechanism that requests clarification for ambiguous commands

## Further Reading

- [Natural Language Processing for Robotics](https://arxiv.org/abs/2003.04620) - Survey of NLP techniques in robotics
- [Grounded Language Understanding](https://arxiv.org/abs/2106.06389) - Connecting language to perception
- [Task Planning in Robotics](https://arxiv.org/abs/2104.05675) - Hierarchical task planning approaches