import Link from "@docusaurus/Link"
import { AuroraText } from "../ui/aurora-text"
import { InteractiveHoverButton } from "../ui/interactive-hover-button"
import { LightRays } from "../ui/light-rays"
import { SparklesText } from "../ui/sparkles-text"

export const HeroSection = () => {
  return (
    <section className="relative flex min-h-screen items-center justify-center overflow-hidden pt-16">
      <LightRays className="z-0" />

      {/* Background gradient */}
      <div className="absolute inset-0 bg-linear-to-b from-transparent via-background/50 to-background" />

      <div className="container relative z-10 mx-auto px-4 text-center">
        <h1 className="mb-6 text-5xl font-bold leading-tight md:text-7xl lg:text-8xl">
          <SparklesText sparkleCount={16}>
            <AuroraText className="font-display">
              Physical AI &<br />Humanoid Robots
            </AuroraText>
          </SparklesText>
        </h1>

        <p className="mx-auto mb-10 max-w-2xl text-xl text-muted-foreground md:text-2xl">
          Bridge the gap between the digital brain and the physical body.
          Master embodied intelligence for the future of AI.
        </p>

        <Link to="/docs/introduction-to-physical-ai-and-embodied-intelligence">
          <InteractiveHoverButton>Start Reading</InteractiveHoverButton>
        </Link>
      </div>

      {/* Bottom fade */}
      <div className="absolute bottom-0 left-0 right-0 h-32 bg-linear-to-t from-background to-transparent" />
    </section >
  )
}