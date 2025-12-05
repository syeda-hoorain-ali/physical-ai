import { BotIcon, BrainIcon, LayersIcon } from "lucide-react";
import { MagicCard } from "../ui/magic-card"

const features = [
  {
    icon: <BotIcon className="h-10 w-10 text-primary" />,
    title: "ROS 2 Mastery",
    description:
      "Master the robotic nervous system with ROS 2 nodes, topics, and services. Bridge Python agents to robot controllers using rclpy.",
  },
  {
    icon: <LayersIcon className="h-10 w-10 text-secondary" />,
    title: "Digital Twin Simulation",
    description:
      "Build physics simulations in Gazebo and Unity. Create high-fidelity environments with LiDAR, depth cameras, and IMU sensors.",
  },
  {
    icon: <BrainIcon className="h-10 w-10 text-primary" />,
    title: "NVIDIA Isaac™ Integration",
    description:
      "Leverage photorealistic simulation, synthetic data generation, and hardware-accelerated VSLAM for advanced AI-robot perception.",
  },
];

export const FeaturesSection = () => {
  return (
    <section className="py-24">
      <div className="container mx-auto px-4">
        <h2 className="mb-4 text-center text-3xl font-bold text-foreground md:text-4xl">
          Core Book Features
        </h2>
        <p className="mx-auto mb-16 max-w-xl text-center text-muted-foreground">
          Everything you need to master Physical AI and Humanoid Robotics
        </p>

        <div className="grid gap-8 md:grid-cols-3">
          {features.map((feature, index) => (
            <MagicCard key={index} className="text-center">
              <div className="mb-4 inline-flex rounded-lg bg-muted p-3">
                {feature.icon}
              </div>
              <h3 className="mb-3 text-xl font-semibold text-foreground">
                {feature.title}
              </h3>
              <p className="text-muted-foreground">{feature.description}</p>
            </MagicCard>
          ))}
        </div>
      </div>
    </section>
  )
}
