import Link from "@docusaurus/Link"
import { InteractiveHoverButton } from "../ui/interactive-hover-button"

export const FooterCTASection = () => {
  return (
    <section className="border-t border-border/30 py-24">
      <div className="container mx-auto px-4 text-center">
        <h2 className="mb-6 text-3xl font-bold text-foreground md:text-4xl">
          Ready to Shape the Future?
        </h2>
        <p className="mx-auto mb-10 max-w-xl text-muted-foreground">
          Start your journey into Physical AI and Humanoid Robotics today.
        </p>
        <Link to="/docs/introduction-to-physical-ai-and-embodied-intelligence">
          <InteractiveHoverButton>Begin Your Journey</InteractiveHoverButton>
        </Link>
      </div>
    </section>
  )
}
