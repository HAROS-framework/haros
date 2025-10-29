# HAROS Metamodel Architecture

This document describes the core data structures and entities that make up the HAROS metamodel, which is used to represent and analyze ROS 2 systems.

## Metamodel Dependency Tree

```
ComputationGraphModel (Runtime System Graph)
├── RosNodeModel[] (Runtime Nodes)
│   ├── RosName (Node Name)
│   ├── Result<Parameters>
│   ├── Result<Remappings>
│   └── Result<Arguments>
│
├── RosLinkModel[] (Communication Links)
│   ├── RosTopicModel[]
│   ├── RosServiceModel[]
│   └── RosActionModel[]
│
└── RosParameterModel[] (System Parameters)

ProjectModel (Static Analysis Root)
├── PackageModel[] (ROS Packages)
│   ├── FileModel[] (Source Files)
│   │   ├── SourceCodeMetadata
│   │   └── SourceCodeDependencies
│   │
│   ├── NodeModel[] (Node Implementation)
│   │   ├── RosClientLibraryCalls
│   │   │   ├── AdvertiseCalls[]
│   │   │   └── SubscribeCalls[]
│   │   │
│   │   ├── SourceCodeMetadata
│   │   └── SourceCodeDependencies
│   │
│   ├── DevelopmentMetadata
│   └── SourceCodeDependencies
│
└── Launch Configuration
    ├── LaunchDescription
    │   ├── LaunchArguments[]
    │   ├── LaunchNodes[]
    │   ├── LaunchInclusions[]
    │   └── LaunchConditions[]
    │
    └── ParameterFiles[]
```

## Core Entities Hierarchy

The metamodel is built on a hierarchy of base entities:

- `RosMetamodelEntity` (base class)
  - `RosFileSystemEntity` - represents files, packages, and nodes on disk
  - `RosSourceEntity` - represents source code elements and client library calls
  - `RosRuntimeEntity` - represents runtime elements like nodes, topics, etc.

## Runtime System (`ComputationGraphModel`)

The runtime graph is the highest level representation of a ROS 2 system:

1. `ComputationGraphModel`

   - Contains runtime nodes and their connections
   - Represents the actual system configuration

2. `RosNodeModel` (Runtime Node)

   - Represents a node instance at runtime
   - Connected to other nodes via topics/services/actions
   - Has parameters and remappings

3. Communication Models
   - `RosLinkModel`: Represents node-to-node communication
   - `RosTopicModel`: Publisher-subscriber connections
   - `RosServiceModel`: Service-client connections
   - `RosActionModel`: Action server-client connections
   - `RosParameterModel`: Configuration parameters

## Static Analysis (`ProjectModel`)

The project model represents the source code and build artifacts:

1. `ProjectModel`

   - Contains all packages and their relationships
   - Maps source files to packages and nodes

2. `PackageModel`

   - Container for related source files and nodes
   - Holds package metadata and dependencies
   - Links to build system configuration

3. `FileModel`

   - Source code files and their metadata
   - Language-specific analysis results
   - Dependencies on other files/packages

4. `NodeModel`
   - Static implementation of a ROS node
   - Contains ROS API usage analysis
   - Links to source files

## Source Code Entities

Supporting types for source code analysis:

1. `RosType`

   - Message/Service/Action type definitions
   - Used in communication interfaces

2. `RosName`

   - Namespace resolution
   - Topic/Service/Action names
   - Parameter names

3. `RosClientLibraryCalls`
   - Analysis of ROS API usage
   - Publisher/Subscriber registration
   - Service/Action declarations

## Launch System

Configuration and runtime initialization:

1. `LaunchDescription`

   - Node instantiation rules
   - Conditional execution
   - Parameter loading
   - Remapping rules

2. `ParameterFiles`
   - YAML configuration files
   - Parameter defaults and overrides

## Builder Components

The `metamodel/builder` package contains modules to construct these entities:

- `files.py` - Builds file models
- `nodes.py` - Builds node models from source code
- `packages.py` - Builds package models
- `projects.py` - Builds complete project models
- `launch.py` - Analyzes launch files and configurations

## Common Features

### Languages

Supported file types:

- C++
- Python
- CMake
- ROS-specific (package.xml, launch, msg, srv, action)
- Other (XML, YAML, JSON)

### Type System

- Built-in type mask system for type analysis
- Support for primitives (bool, int, double, string)
- Support for containers (lists, mappings)

### Analysis Support

- Source code location tracking
- Dependency tracking
- Build and runtime metadata
- Error and result handling

## Static Analysis Core Concepts

### Handling Uncertainty with `Result`

The `Result[T]` class is fundamental to HAROS's static analysis capabilities. Its primary purpose is to represent the uncertainty inherent in static analysis:

- **Known vs Unknown Values**: When analyzing source code, many values cannot be determined with certainty until runtime. `Result[T]` explicitly models this uncertainty:

  - Known values: `Result.of(value)`
  - Unknown values: `Result.unknown_value()`
  - Partially known values: e.g., "some string starting with 'log\_'"

- **Usage in Models**: Many model properties are wrapped in `Result` to indicate our confidence in the analysis:
  ```python
  class RosNodeModel:
      rosname: Result[RosName]        # May be unknown at analysis time
      parameters: Result[dict]        # Values may be partially known
      arguments: Result[list[str]]    # May depend on launch configuration
  ```

### Conditional Values with `VariantValue` and `VariantData`

The `VariantValue` and `VariantData` classes represent values that depend on runtime conditions, typically from control flow analysis:

- **Conditional Values**: `VariantValue[T]` pairs a value with a logical condition under which it holds:

  ```python
  # Representing a variable after an if-else block
  x = VariantData[Result[int]]([
      VariantValue(Result.of(1), condition="a > 0"),
      VariantValue(Result.of(2), condition="a <= 0")
  ])
  ```

- **Complex Control Flow**: `VariantData[T]` manages multiple possible values:

  - Different values from different code paths
  - Tracks logical conditions for each variant
  - Maintains a base/default value when no conditions match

- **Combination with Results**: Often used as `VariantData[Result[T]]` to represent both:
  - Uncertainty from static analysis (`Result`)
  - Multiple possibilities from control flow (`VariantData`)

Example analysis of a Python function:

```python
def configure_logger(level):
    if level == "debug":
        path = "/var/log/debug.log"
    else:
        path = "/var/log/info.log"
    return path

# Represented in HAROS as:
path = VariantData[Result[str]]([
    VariantValue(Result.of("/var/log/debug.log"),
                condition="level == 'debug'"),
    VariantValue(Result.of("/var/log/info.log"),
                condition="level != 'debug'")
])
```

## Notes

- Entities are generally immutable (using `@frozen` decorator)
- Extensive use of `Result` and `VariantData` for accurate static analysis
- Builder pattern used for constructing complex objects
- Extensive validation at boundaries
