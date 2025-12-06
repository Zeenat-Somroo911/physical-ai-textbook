---
sidebar_position: 5
---

# Chapter 4: Action Prediction

## Introduction

Action prediction is the final stage of VLA systems, generating robot actions from visual observations and language instructions.

## Action Spaces

### Discrete Actions

```python
class DiscreteActionDecoder(nn.Module):
    def __init__(self, num_actions):
        super().__init__()
        self.action_head = nn.Linear(hidden_dim, num_actions)
    
    def forward(self, features):
        logits = self.action_head(features)
        action = torch.argmax(logits, dim=-1)
        return action
```

### Continuous Actions

```python
class ContinuousActionDecoder(nn.Module):
    def __init__(self, action_dim):
        super().__init__()
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.std_head = nn.Linear(hidden_dim, action_dim)
    
    def forward(self, features):
        mean = self.mean_head(features)
        std = F.softplus(self.std_head(features)) + 1e-5
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        return action, dist
```

## Action Prediction Architectures

### Direct Prediction

```python
class DirectActionPredictor(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = MultimodalEncoder()
        self.action_decoder = ActionDecoder()
    
    def forward(self, image, instruction):
        features = self.encoder(image, instruction)
        action = self.action_decoder(features)
        return action
```

### Hierarchical Prediction

```python
class HierarchicalActionPredictor(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = MultimodalEncoder()
        self.high_level = HighLevelPlanner()
        self.low_level = LowLevelController()
    
    def forward(self, image, instruction):
        features = self.encoder(image, instruction)
        
        # High-level: "pick up cup"
        high_level_action = self.high_level(features)
        
        # Low-level: joint angles, velocities
        low_level_action = self.low_level(features, high_level_action)
        return low_level_action
```

## Imitation Learning

### Behavioral Cloning

```python
def behavioral_cloning_loss(predicted_actions, expert_actions):
    loss = F.mse_loss(predicted_actions, expert_actions)
    return loss

# Training
for image, instruction, expert_action in dataloader:
    predicted = model(image, instruction)
    loss = behavioral_cloning_loss(predicted, expert_action)
    loss.backward()
```

### Dataset Aggregation (DAgger)

```python
def dagger_training(model, expert_policy):
    dataset = initial_dataset
    
    for iteration in range(num_iterations):
        # Train on current dataset
        train(model, dataset)
        
        # Collect new data with current policy
        new_data = []
        for state in environment:
            action = model(state)
            expert_action = expert_policy(state)
            new_data.append((state, expert_action))
        
        # Aggregate datasets
        dataset = dataset + new_data
```

## Reinforcement Learning

### Policy Gradient

```python
def policy_gradient_loss(actions, rewards, log_probs):
    # REINFORCE
    returns = compute_returns(rewards)
    loss = -torch.mean(log_probs * returns)
    return loss
```

### Actor-Critic

```python
class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor = ActionDecoder()
        self.critic = ValueEstimator()
    
    def forward(self, image, instruction):
        features = self.encoder(image, instruction)
        action_dist = self.actor(features)
        value = self.critic(features)
        return action_dist, value
```

## Action Chunking

### Predicting Action Sequences

```python
class ActionChunking(nn.Module):
    def __init__(self, chunk_size=10):
        super().__init__()
        self.encoder = MultimodalEncoder()
        self.decoder = nn.LSTM(hidden_dim, action_dim, num_layers=2)
    
    def forward(self, image, instruction):
        features = self.encoder(image, instruction)
        
        # Predict sequence of actions
        hidden = features.unsqueeze(0)
        actions = []
        for _ in range(chunk_size):
            action, hidden = self.decoder(hidden)
            actions.append(action)
        
        return torch.stack(actions)
```

## Safety and Constraints

### Action Constraints

```python
class ConstrainedActionDecoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.action_decoder = ActionDecoder()
    
    def forward(self, features, constraints):
        raw_action = self.action_decoder(features)
        
        # Apply constraints
        constrained_action = apply_constraints(
            raw_action,
            max_velocity=constraints['max_vel'],
            joint_limits=constraints['joint_limits']
        )
        return constrained_action
```

## Evaluation Metrics

- **Success Rate**: Task completion percentage
- **Path Length**: Efficiency of actions
- **Smoothness**: Action trajectory smoothness
- **Safety**: Constraint violations

## Best Practices

- Use appropriate action representation
- Consider temporal dependencies
- Implement safety constraints
- Balance exploration and exploitation
- Test in simulation before real robot

## Exercises

1. Implement action decoder
2. Train with imitation learning
3. Add action constraints

## Next Steps

Chapter 5 covers building end-to-end VLA systems.

